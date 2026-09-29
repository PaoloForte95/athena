package org.pofe.athena.optimizer;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.logging.Logger;

import org.pofe.athena.util.Logging;

import com.google.ortools.Loader;
import com.google.ortools.sat.BoolVar;
import com.google.ortools.sat.CircuitConstraint;
import com.google.ortools.sat.CpModel;
import com.google.ortools.sat.CpSolver;
import com.google.ortools.sat.CpSolverStatus;
import com.google.ortools.sat.DoubleLinearExpr;
import com.google.ortools.sat.IntVar;
import com.google.ortools.sat.LinearExpr;
import com.google.ortools.sat.LinearExprBuilder;

public class JobAssignment {

    public interface CostModel {

        default long duration(String robot, Job job) {
            return Math.max(1, job.getActions().size());
        }

        default long travelTime(String robot, String from, String to) {
            if (from == null || to == null || from.isEmpty() || to.isEmpty() || from.equals(to)) {
                return 0;
            }
            return 1;
        }

        default double travelCost(String robot, String from, String to) {
            return travelTime(robot, from, to);
        }

        default double assignmentCost(String robot, Job job) {
            return 0.0;
        }
    }

    public static final class Result {
        private final CpSolverStatus status;
        private final Map<String, List<Job>> sequences;
        private final Map<Integer, Long> startTimes;
        private final Map<Integer, Long> endTimes;
        private final long makespan;
        private final double objective;

        Result(CpSolverStatus status, Map<String, List<Job>> sequences, Map<Integer, Long> startTimes,
               Map<Integer, Long> endTimes, long makespan, double objective) {
            this.status = status;
            this.sequences = sequences;
            this.startTimes = startTimes;
            this.endTimes = endTimes;
            this.makespan = makespan;
            this.objective = objective;
        }

        public CpSolverStatus getStatus() {
            return status;
        }

        public boolean hasSolution() {
            return status == CpSolverStatus.OPTIMAL || status == CpSolverStatus.FEASIBLE;
        }

        public boolean isOptimal() {
            return status == CpSolverStatus.OPTIMAL;
        }

        public Map<String, List<Job>> getSequences() {
            return Collections.unmodifiableMap(sequences);
        }

        public String getRobot(Job job) {
            for (Map.Entry<String, List<Job>> entry : sequences.entrySet()) {
                if (entry.getValue().contains(job)) {
                    return entry.getKey();
                }
            }
            return null;
        }

        public long getStartTime(Job job) {
            return startTimes.getOrDefault(job.getId(), -1L);
        }

        public long getEndTime(Job job) {
            return endTimes.getOrDefault(job.getId(), -1L);
        }

        public long getMakespan() {
            return makespan;
        }

        public double getObjective() {
            return objective;
        }

        public boolean isChanged() {
            for (Map.Entry<String, List<Job>> entry : sequences.entrySet()) {
                for (Job job : entry.getValue()) {
                    if (!entry.getKey().equals(job.getRobot())) {
                        return true;
                    }
                }
            }
            return false;
        }

        @Override
        public String toString() {
            StringBuilder builder = new StringBuilder("Status ").append(status);
            if (!hasSolution()) {
                return builder.toString();
            }
            builder.append(", makespan ").append(makespan).append(", cost ").append(objective);
            for (Map.Entry<String, List<Job>> entry : sequences.entrySet()) {
                builder.append("\n  ").append(entry.getKey()).append(":");
                if (entry.getValue().isEmpty()) {
                    builder.append(" -");
                }
                for (Job job : entry.getValue()) {
                    builder.append(" job ").append(job.getId())
                            .append(" [").append(getStartTime(job)).append("-").append(getEndTime(job)).append("]");
                }
            }
            return builder.toString();
        }
    }

    private static final class Arc {
        private final int tail;
        private final int head;
        private final BoolVar literal;

        Arc(int tail, int head, BoolVar literal) {
            this.tail = tail;
            this.head = head;
            this.literal = literal;
        }
    }

    private static final Logger logger = Logging.getLogger(JobAssignment.class);
    private static boolean nativeLibrariesLoaded = false;

    private final Optimizer optimizer;
    private final CostModel costModel;
    private final List<Job> jobs;
    private final List<String> robots;
    private final CpModel model = new CpModel();
    private final List<IntVar> objectiveVariables = new ArrayList<>();
    private final List<Double> objectiveCoefficients = new ArrayList<>();
    private final Map<String, List<Arc>> robotArcs = new HashMap<>();
    private final Map<String, int[]> robotNodeJobs = new HashMap<>();
    private BoolVar[][] present;
    private IntVar[] start;
    private IntVar[] end;
    private IntVar makespan;
    private double makespanWeight = 1.0;
    private double travelWeight = 0.0;
    private double assignmentWeight = 0.0;
    private double changeWeight = 0.001;
    private double timeLimit = 10.0;
    private boolean built = false;
    private Result lastResult;

    public JobAssignment(Optimizer optimizer, CostModel costModel) {
        loadNativeLibraries();
        this.optimizer = optimizer;
        this.costModel = costModel;
        this.jobs = new ArrayList<>(optimizer.getJobs());
        this.robots = new ArrayList<>(optimizer.getRobots());
    }

    private static synchronized void loadNativeLibraries() {
        if (!nativeLibrariesLoaded) {
            Loader.loadNativeLibraries();
            nativeLibrariesLoaded = true;
        }
    }

    public void setWeights(double makespanWeight, double travelWeight, double assignmentWeight) {
        checkNotBuilt();
        this.makespanWeight = makespanWeight;
        this.travelWeight = travelWeight;
        this.assignmentWeight = assignmentWeight;
    }

    public void setChangeWeight(double changeWeight) {
        checkNotBuilt();
        this.changeWeight = changeWeight;
    }

    public void setTimeLimit(double seconds) {
        this.timeLimit = seconds;
    }

    public Result getLastResult() {
        return lastResult;
    }

    public Result solve() {
        if (!built) {
            build();
        }
        CpSolver solver = new CpSolver();
        solver.getParameters().setMaxTimeInSeconds(timeLimit);
        CpSolverStatus status = solver.solve(model);
        if (status != CpSolverStatus.OPTIMAL && status != CpSolverStatus.FEASIBLE) {
            lastResult = new Result(status, new LinkedHashMap<>(), new HashMap<>(), new HashMap<>(), -1, Double.NaN);
            return lastResult;
        }
        Map<String, List<Job>> sequences = new LinkedHashMap<>();
        for (String robot : robots) {
            sequences.put(robot, extractSequence(solver, robot));
        }
        Map<Integer, Long> startTimes = new HashMap<>();
        Map<Integer, Long> endTimes = new HashMap<>();
        for (int j = 0; j < jobs.size(); j++) {
            startTimes.put(jobs.get(j).getId(), solver.value(start[j]));
            endTimes.put(jobs.get(j).getId(), solver.value(end[j]));
        }
        lastResult = new Result(status, sequences, startTimes, endTimes, solver.value(makespan), solver.objectiveValue());
        return lastResult;
    }

    public void forbidLastAssignment() {
        if (lastResult == null || !lastResult.hasSolution()) {
            return;
        }
        LinearExprBuilder chosen = LinearExpr.newBuilder();
        int count = 0;
        for (int r = 0; r < robots.size(); r++) {
            for (Job job : lastResult.getSequences().get(robots.get(r))) {
                chosen.add(present[r][jobs.indexOf(job)]);
                count++;
            }
        }
        model.addLessOrEqual(chosen, count - 1);
        model.clearHints();
    }

    private void checkNotBuilt() {
        if (built) {
            throw new IllegalStateException("The model is already built; set the weights before solving");
        }
    }

    private void build() {
        int n = jobs.size();
        int m = robots.size();
        long horizon = computeHorizon();
        start = new IntVar[n];
        end = new IntVar[n];
        present = new BoolVar[m][n];
        makespan = model.newIntVar(0, horizon, "makespan");
        addObjectiveTerm(makespan, makespanWeight);

        for (int j = 0; j < n; j++) {
            start[j] = model.newIntVar(0, horizon, "start_" + jobs.get(j).getId());
            end[j] = model.newIntVar(0, horizon, "end_" + jobs.get(j).getId());
            model.addGreaterOrEqual(end[j], start[j]);
            model.addGreaterOrEqual(makespan, end[j]);
        }

        for (int j = 0; j < n; j++) {
            Job job = jobs.get(j);
            List<BoolVar> candidates = new ArrayList<>();
            for (int r = 0; r < m; r++) {
                String robot = robots.get(r);
                if (!optimizer.canDo(robot, job)) {
                    continue;
                }
                present[r][j] = model.newBoolVar("present_" + robot + "_" + job.getId());
                candidates.add(present[r][j]);
                model.addEquality(end[j], plus(start[j], costModel.duration(robot, job))).onlyEnforceIf(present[r][j]);
                model.addHint(present[r][j], robot.equals(job.getRobot()));
                addObjectiveTerm(present[r][j], assignmentWeight * costModel.assignmentCost(robot, job));
                if (!robot.equals(job.getRobot())) {
                    addObjectiveTerm(present[r][j], changeWeight);
                }
            }
            if (candidates.isEmpty()) {
                logger.warning("No robot is able to do job " + job.getId() + ": " + job.getRequires());
            }
            model.addExactlyOne(new ArrayList<>(candidates));
        }

        for (int r = 0; r < m; r++) {
            buildCircuit(r);
        }

        Map<Integer, Integer> jobIndex = new HashMap<>();
        for (int j = 0; j < n; j++) {
            jobIndex.put(jobs.get(j).getId(), j);
        }
        for (int j = 0; j < n; j++) {
            for (int predecessor : jobs.get(j).getPredecessors()) {
                Integer p = jobIndex.get(predecessor);
                if (p != null) {
                    model.addGreaterOrEqual(start[j], end[p]);
                }
            }
        }

        IntVar[] variables = objectiveVariables.toArray(new IntVar[0]);
        double[] coefficients = new double[objectiveCoefficients.size()];
        for (int i = 0; i < coefficients.length; i++) {
            coefficients[i] = objectiveCoefficients.get(i);
        }
        model.minimize(DoubleLinearExpr.weightedSum(variables, coefficients));
        built = true;
    }

    private void buildCircuit(int r) {
        String robot = robots.get(r);
        List<Integer> allowed = new ArrayList<>();
        for (int j = 0; j < jobs.size(); j++) {
            if (present[r][j] != null) {
                allowed.add(j);
            }
        }
        if (allowed.isEmpty()) {
            return;
        }
        Set<String> initialState = optimizer.getInitialStates().get(robot);
        String home = optimizer.getInitialLocationName(robot);
        CircuitConstraint circuit = model.addCircuit();
        List<Arc> arcs = new ArrayList<>();
        int[] nodeJobs = new int[allowed.size() + 1];
        nodeJobs[0] = -1;
        circuit.addArc(0, 0, model.newBoolVar("unused_" + robot));

        for (int k = 0; k < allowed.size(); k++) {
            int j = allowed.get(k);
            Job job = jobs.get(j);
            int node = k + 1;
            nodeJobs[node] = j;
            circuit.addArc(node, node, present[r][j].not());
            BoolVar last = model.newBoolVar("last_" + robot + "_" + job.getId());
            circuit.addArc(node, 0, last);
            arcs.add(new Arc(node, 0, last));
            if (job.getStartState().equals(initialState)) {
                BoolVar first = model.newBoolVar("first_" + robot + "_" + job.getId());
                circuit.addArc(0, node, first);
                arcs.add(new Arc(0, node, first));
                model.addGreaterOrEqual(start[j], costModel.travelTime(robot, home, job.getStartLocation()))
                        .onlyEnforceIf(first);
                addObjectiveTerm(first, travelWeight * costModel.travelCost(robot, home, job.getStartLocation()));
            }
            for (int k2 = 0; k2 < allowed.size(); k2++) {
                int i = allowed.get(k2);
                Job previous = jobs.get(i);
                if (i == j || !previous.getEndState().equals(job.getStartState())) {
                    continue;
                }
                BoolVar arc = model.newBoolVar("arc_" + robot + "_" + previous.getId() + "_" + job.getId());
                circuit.addArc(k2 + 1, node, arc);
                arcs.add(new Arc(k2 + 1, node, arc));
                long travel = costModel.travelTime(robot, previous.getEndLocation(), job.getStartLocation());
                model.addGreaterOrEqual(start[j], plus(end[i], travel)).onlyEnforceIf(arc);
                addObjectiveTerm(arc, travelWeight
                        * costModel.travelCost(robot, previous.getEndLocation(), job.getStartLocation()));
            }
        }
        robotArcs.put(robot, arcs);
        robotNodeJobs.put(robot, nodeJobs);
    }

    private List<Job> extractSequence(CpSolver solver, String robot) {
        List<Job> sequence = new ArrayList<>();
        List<Arc> arcs = robotArcs.get(robot);
        if (arcs == null) {
            return sequence;
        }
        Map<Integer, Integer> next = new HashMap<>();
        for (Arc arc : arcs) {
            if (solver.booleanValue(arc.literal)) {
                next.put(arc.tail, arc.head);
            }
        }
        int[] nodeJobs = robotNodeJobs.get(robot);
        Integer node = next.get(0);
        while (node != null && node != 0 && sequence.size() < nodeJobs.length) {
            sequence.add(jobs.get(nodeJobs[node]));
            node = next.get(node);
        }
        return sequence;
    }

    private long computeHorizon() {
        long horizon = 0;
        for (Job job : jobs) {
            long maxDuration = 0;
            long maxTravel = 0;
            for (String robot : robots) {
                if (!optimizer.canDo(robot, job)) {
                    continue;
                }
                maxDuration = Math.max(maxDuration, costModel.duration(robot, job));
                maxTravel = Math.max(maxTravel,
                        costModel.travelTime(robot, optimizer.getInitialLocationName(robot), job.getStartLocation()));
                for (Job previous : jobs) {
                    if (previous != job) {
                        maxTravel = Math.max(maxTravel,
                                costModel.travelTime(robot, previous.getEndLocation(), job.getStartLocation()));
                    }
                }
            }
            horizon += maxDuration + maxTravel;
        }
        return Math.max(horizon, 1);
    }

    private void addObjectiveTerm(IntVar variable, double coefficient) {
        if (coefficient != 0.0) {
            objectiveVariables.add(variable);
            objectiveCoefficients.add(coefficient);
        }
    }

    private static LinearExprBuilder plus(IntVar variable, long constant) {
        return LinearExpr.newBuilder().add(variable).add(constant);
    }
}
