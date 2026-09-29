package org.pofe.athena.optimizer;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;
import java.util.logging.Logger;

import org.pofe.athena.parser.Action;
import org.pofe.athena.parser.SymbolicSymbol;
import org.pofe.athena.problems.PlanningProblem;
import org.pofe.athena.util.Logging;

import fr.uga.pddl4j.parser.Connector;
import fr.uga.pddl4j.parser.Expression;
import fr.uga.pddl4j.parser.Symbol;
import fr.uga.pddl4j.parser.TypedSymbol;

public class Optimizer {

    public static final String ROBOT_PLACEHOLDER = "?r";

    private static final Logger logger = Logging.getLogger(Optimizer.class);

    private final Set<String> robotDefinitions = new HashSet<>();
    private final Set<String> movementActions = new HashSet<>();
    private final List<String> robots = new ArrayList<>();
    private final List<Action> plan = new ArrayList<>();
    private final List<Action> liftedPlan = new ArrayList<>();
    private final List<Job> jobs = new ArrayList<>();
    private final List<Job> actionJobs = new ArrayList<>();
    private final List<Integer> precedenceCycle = new ArrayList<>();
    private final List<Action> actionsWithoutRobot = new ArrayList<>();
    private final Set<String> dynamicPredicates = new HashSet<>();
    private final Set<String> locationPredicates = new HashSet<>();
    private final Map<String, Set<String>> initialStates = new LinkedHashMap<>();
    private final Map<String, Set<String>> initialLocations = new LinkedHashMap<>();
    private final Map<String, Set<String>> capabilities = new LinkedHashMap<>();
    private PlanningProblem problem;
    private JobAssignment.CostModel costModel = new JobAssignment.CostModel() {};
    private double makespanWeight = 1.0;
    private double travelWeight = 0.0;
    private double assignmentWeight = 0.0;
    private double changeWeight = 0.001;
    private double timeLimit = 10.0;
    private int maxAttempts = 10;
    private JobAssignment assignment;
    private JobAssignment.Result assignmentResult;

    public Optimizer(Collection<String> robotDefinitions, Collection<String> movementActions) {
        for (String definition : robotDefinitions) {
            this.robotDefinitions.add(normalize(definition));
        }
        for (String movement : movementActions) {
            this.movementActions.add(normalize(movement));
        }
    }

    public File optimize(PlanningProblem problem, File planFile) throws IOException {
        if (problem == null || problem.getDomain() == null) {
            throw new IllegalArgumentException("The planning problem must be parsed before the optimization");
        }
        this.problem = problem;
        buildRobots();
        readPlan(planFile);
        liftedPlan.clear();
        for (Action action : plan) {
            liftedPlan.add(lift(action));
        }
        buildPredicates();
        buildInitialStates();
        buildJobs();
        describeJobs();
        buildPrecedences();
        findPrecedenceCycle();
        solveAssignment();
        File optimalPlanFile = getOptimalPlanFile(planFile);
        writePlan(buildOptimalPlan(), optimalPlanFile);
        return optimalPlanFile;
    }

    public static File getOptimalPlanFile(File planFile) {
        String name = planFile.getName();
        int dot = name.lastIndexOf('.');
        String optimalName = dot > 0
                ? name.substring(0, dot) + "_optimal" + name.substring(dot)
                : name + "_optimal";
        return new File(planFile.getAbsoluteFile().getParentFile(), optimalName);
    }

    public PlanningProblem getProblem() {
        return problem;
    }

    public void setCostModel(JobAssignment.CostModel costModel) {
        this.costModel = costModel;
    }

    public void setWeights(double makespanWeight, double travelWeight, double assignmentWeight) {
        this.makespanWeight = makespanWeight;
        this.travelWeight = travelWeight;
        this.assignmentWeight = assignmentWeight;
    }

    public void setChangeWeight(double changeWeight) {
        this.changeWeight = changeWeight;
    }

    public void setTimeLimit(double seconds) {
        this.timeLimit = seconds;
    }

    public void setMaxAttempts(int maxAttempts) {
        this.maxAttempts = Math.max(1, maxAttempts);
    }

    public JobAssignment getAssignment() {
        return assignment;
    }

    public JobAssignment.Result getAssignmentResult() {
        return assignmentResult;
    }

    public String getInitialLocationName(String robot) {
        Set<String> location = initialLocations.get(normalize(robot));
        return location == null ? "" : locationName(location);
    }

    public List<Action> getPlan() {
        return Collections.unmodifiableList(plan);
    }

    public List<Action> getLiftedPlan() {
        return Collections.unmodifiableList(liftedPlan);
    }

    public List<Job> getJobs() {
        return Collections.unmodifiableList(jobs);
    }

    public List<Action> getActionsWithoutRobot() {
        return Collections.unmodifiableList(actionsWithoutRobot);
    }

    public Map<String, Set<String>> getInitialStates() {
        return Collections.unmodifiableMap(initialStates);
    }

    public Map<String, Set<String>> getInitialLocations() {
        return Collections.unmodifiableMap(initialLocations);
    }

    public Map<String, Set<String>> getCapabilities() {
        return Collections.unmodifiableMap(capabilities);
    }

    public boolean canDo(String robot, Job job) {
        Set<String> robotCapabilities = capabilities.get(normalize(robot));
        return robotCapabilities != null && robotCapabilities.containsAll(job.getRequires());
    }

    public boolean hasPrecedenceCycle() {
        return !precedenceCycle.isEmpty();
    }

    public List<Integer> getPrecedenceCycle() {
        return Collections.unmodifiableList(precedenceCycle);
    }

    public boolean isRobotType(Collection<String> types) {
        if (types == null) {
            return false;
        }
        Set<String> normalized = new HashSet<>();
        for (String type : types) {
            normalized.add(normalize(type));
        }
        return !Collections.disjoint(normalized, robotDefinitions);
    }

    public boolean isRobot(SymbolicSymbol input) {
        return input != null && isRobotType(input.getType());
    }

    public boolean isMovement(Action action) {
        return movementActions.contains(normalize(action.getName()));
    }

    public List<String> getRobots(Action action) {
        List<String> actionRobots = new ArrayList<>();
        for (SymbolicSymbol input : action.getInputs()) {
            if (isRobot(input) && !actionRobots.contains(normalize(input.getVariable()))) {
                actionRobots.add(normalize(input.getVariable()));
            }
        }
        return actionRobots;
    }

    public List<String> getRobots() {
        return Collections.unmodifiableList(robots);
    }

    public Set<String> getPlanRobots() {
        Set<String> planRobots = new LinkedHashSet<>();
        for (Action action : plan) {
            planRobots.addAll(getRobots(action));
        }
        return planRobots;
    }

    public boolean isMultiRobot(Action action) {
        return getRobots(action).size() > 1;
    }

    private void buildRobots() {
        robots.clear();
        for (TypedSymbol<String> object : problem.getProblem().getObjects()) {
            String name = normalize(object.getImage().toString());
            if (isRobotType(problem.getType(name)) && !robots.contains(name)) {
                robots.add(name);
            }
        }
    }

    private void readPlan(File planFile) throws IOException {
        plan.clear();
        Set<String> actionNames = problem.getActionsNames();
        try (BufferedReader reader = Files.newBufferedReader(planFile.toPath(), StandardCharsets.UTF_8)) {
            String line;
            while ((line = reader.readLine()) != null) {
                Action action = parseLine(line, actionNames);
                if (action != null) {
                    plan.add(action);
                }
            }
        }
    }

    private Action parseLine(String line, Set<String> actionNames) {
        String text = line.trim().toLowerCase(Locale.ROOT);
        if (text.isEmpty() || text.startsWith(";")) {
            return null;
        }
        text = text.replaceAll("\\[[^\\]]*\\]", " ").replaceAll("[()]", " ").trim();
        String[] tokens = text.split("\\s+");
        int nameIndex = -1;
        for (int i = 0; i < tokens.length; i++) {
            if (actionNames.contains(tokens[i])) {
                nameIndex = i;
                break;
            }
        }
        if (nameIndex < 0) {
            return null;
        }
        Action action = problem.getAction(tokens[nameIndex]);
        action.setID(plan.size());
        List<String> parameters = action.getParameters();
        int argumentCount = tokens.length - nameIndex - 1;
        if (argumentCount != parameters.size()) {
            throw new IllegalArgumentException("Action " + tokens[nameIndex] + " expects " + parameters.size()
                    + " arguments but the plan gives " + argumentCount + ": " + line);
        }
        for (int i = 0; i < parameters.size(); i++) {
            String object = tokens[nameIndex + 1 + i];
            action.addInput(parameters.get(i), new SymbolicSymbol(object, problem.getType(object)));
        }
        return action;
    }

    private void writePlan(List<Action> actions, File file) throws IOException {
        try (BufferedWriter writer = Files.newBufferedWriter(file.toPath(), StandardCharsets.UTF_8)) {
            for (int i = 0; i < actions.size(); i++) {
                writer.write(i + ": " + planLine(actions.get(i)));
                writer.newLine();
            }
        }
    }

    private static String planLine(Action action) {
        StringBuilder builder = new StringBuilder(action.getName());
        for (String parameter : action.getParameters()) {
            SymbolicSymbol input = action.getInput(parameter);
            if (input != null) {
                builder.append(' ').append(input.getVariable());
            }
        }
        return builder.toString();
    }

    private Action lift(Action action) {
        List<String> actionRobots = getRobots(action);
        Action lifted = action.getCopy();
        lifted.setID(action.getID());
        for (String parameter : action.getParameters()) {
            SymbolicSymbol input = action.getInput(parameter);
            if (input == null) {
                continue;
            }
            int index = actionRobots.indexOf(normalize(input.getVariable()));
            if (index < 0) {
                lifted.addInput(parameter, input);
            } else {
                String placeholder = actionRobots.size() == 1 ? ROBOT_PLACEHOLDER : ROBOT_PLACEHOLDER + (index + 1);
                lifted.addInput(parameter, new SymbolicSymbol(placeholder, input.getType()));
            }
        }
        return lifted;
    }

    private void buildPredicates() {
        dynamicPredicates.clear();
        locationPredicates.clear();
        for (Action schema : problem.getActions().values()) {
            for (Expression<String> effect : schema.getEffects()) {
                Expression<String> atom = atomOf(effect);
                if (atom != null) {
                    dynamicPredicates.add(predicateOf(atom));
                }
            }
        }
        for (Action action : liftedPlan) {
            if (!isMovement(action)) {
                continue;
            }
            for (Expression<String> effect : action.getEffects()) {
                Expression<String> atom = atomOf(effect);
                if (atom != null && containsRobot(atom)) {
                    locationPredicates.add(predicateOf(atom));
                }
            }
        }
    }

    private void buildInitialStates() {
        initialStates.clear();
        initialLocations.clear();
        capabilities.clear();
        for (String robot : robots) {
            Set<String> state = new TreeSet<>();
            Set<String> location = new TreeSet<>();
            Set<String> robotCapabilities = new TreeSet<>();
            for (Expression<String> fact : problem.getProblem().getInit()) {
                if (fact.getConnector() != Connector.ATOM || !hasArgument(fact, robot)) {
                    continue;
                }
                String predicate = predicateOf(fact);
                if (locationPredicates.contains(predicate)) {
                    location.add(factKey(fact, robot));
                } else if (dynamicPredicates.contains(predicate)) {
                    state.add(factKey(fact, robot));
                } else {
                    robotCapabilities.add(factKey(fact, robot));
                }
            }
            initialStates.put(robot, state);
            initialLocations.put(robot, location);
            capabilities.put(robot, robotCapabilities);
        }
    }

    private void buildJobs() {
        jobs.clear();
        actionJobs.clear();
        actionsWithoutRobot.clear();
        Map<String, Set<String>> states = new HashMap<>();
        Map<String, Set<String>> locations = new HashMap<>();
        Map<String, Job> openJobs = new LinkedHashMap<>();
        for (String robot : initialStates.keySet()) {
            states.put(robot, new TreeSet<>(initialStates.get(robot)));
            locations.put(robot, new TreeSet<>(initialLocations.get(robot)));
        }
        for (int i = 0; i < plan.size(); i++) {
            Action original = plan.get(i);
            Action lifted = liftedPlan.get(i);
            List<String> actionRobots = getRobots(original);
            if (actionRobots.isEmpty()) {
                actionsWithoutRobot.add(original);
                actionJobs.add(null);
                continue;
            }
            if (actionRobots.size() > 1) {
                throw new UnsupportedOperationException("Actions with more than one robot are not supported yet: " + original);
            }
            String robot = actionRobots.get(0);
            if (!states.containsKey(robot)) {
                throw new IllegalArgumentException("Robot " + robot + " is used in the plan but not declared in the problem");
            }
            Set<String> state = states.get(robot);
            Set<String> location = locations.get(robot);
            Job job = openJobs.get(robot);
            if (isMovement(lifted)) {
                if (job == null || job.hasActions()) {
                    job = openJob(robot, job, state, location, openJobs);
                }
                job.addMovement(lifted);
            } else {
                if (job == null || (job.hasActions() && state.equals(initialStates.get(robot)))) {
                    job = openJob(robot, job, state, location, openJobs);
                }
                if (!job.hasActions()) {
                    job.setStartLocation(locationName(location), location);
                }
                job.addAction(lifted);
            }
            actionJobs.add(job);
            applyEffects(lifted, state, location);
        }
        for (Map.Entry<String, Job> entry : openJobs.entrySet()) {
            Set<String> location = locations.get(entry.getKey());
            entry.getValue().close(states.get(entry.getKey()), locationName(location), location);
        }
    }

    private Job openJob(String robot, Job previous, Set<String> state, Set<String> location, Map<String, Job> openJobs) {
        if (previous != null) {
            previous.close(state, locationName(location), location);
        }
        Job job = new Job(jobs.size(), robot, state);
        jobs.add(job);
        openJobs.put(robot, job);
        return job;
    }

    private void solveAssignment() {
        assignment = null;
        assignmentResult = null;
        if (jobs.isEmpty() || hasPrecedenceCycle()) {
            return;
        }
        assignment = new JobAssignment(this, costModel);
        assignment.setWeights(makespanWeight, travelWeight, assignmentWeight);
        assignment.setChangeWeight(changeWeight);
        assignment.setTimeLimit(timeLimit);
        assignmentResult = assignment.solve();
        if (assignmentResult.hasSolution()) {
            logger.info("Job assignment: " + assignmentResult);
        } else {
            logger.warning("No job assignment found (" + assignmentResult.getStatus() + "). The original plan is kept.");
        }
    }

    private List<Action> buildOptimalPlan() {
        if (assignment == null || assignmentResult == null || !assignmentResult.hasSolution()) {
            return plan;
        }
        if (!actionsWithoutRobot.isEmpty()) {
            logger.warning("The plan has actions without a robot: " + actionsWithoutRobot + ". The original plan is kept.");
            return plan;
        }
        JobAssignment.Result result = assignmentResult;
        for (int attempt = 1; attempt <= maxAttempts; attempt++) {
            if (!result.isChanged()) {
                logger.info("The original assignment is already the best one. The original plan is kept.");
                return plan;
            }
            List<Action> rebuilt = new ArrayList<>();
            String problemFound = rebuildPlan(result, rebuilt);
            if (problemFound == null) {
                logger.info("New plan with " + rebuilt.size() + " actions and makespan " + result.getMakespan() + ".");
                return rebuilt;
            }
            logger.warning("Assignment rejected (attempt " + attempt + "): " + problemFound);
            assignment.forbidLastAssignment();
            result = assignment.solve();
            assignmentResult = result;
            if (!result.hasSolution()) {
                break;
            }
            logger.info("Job assignment: " + result);
        }
        logger.warning("No valid reassignment found. The original plan is kept.");
        return plan;
    }

    private String rebuildPlan(JobAssignment.Result result, List<Action> rebuilt) {
        Set<String> state = initialFacts();
        List<Job> ordered = new ArrayList<>(jobs);
        ordered.sort((a, b) -> {
            int byTime = Long.compare(result.getStartTime(a), result.getStartTime(b));
            return byTime != 0 ? byTime : Integer.compare(a.getId(), b.getId());
        });
        Action movementTemplate = movementTemplate();
        for (Job job : ordered) {
            String robot = result.getRobot(job);
            if (robot == null) {
                return "job " + job.getId() + " has no robot";
            }
            List<Action> actions = new ArrayList<>();
            Set<String> current = robotLocation(state, robot);
            Set<String> target = job.getStartLocationFacts();
            List<Action> movements = job.getMovements();
            if (!target.isEmpty() && !current.equals(target)) {
                if (!movements.isEmpty() && current.equals(fromFacts(movements.get(0)))) {
                    for (Action movement : movements) {
                        actions.add(ground(movement, robot, Collections.emptyMap()));
                    }
                } else {
                    Action template = movements.isEmpty() ? movementTemplate : movements.get(0);
                    if (template == null) {
                        return "no movement action can bring " + robot + " to " + job.getStartLocation();
                    }
                    Map<String, String> replacements = locationReplacements(fromFacts(template), current);
                    if (movements.isEmpty()) {
                        replacements.putAll(locationReplacements(toFacts(template), target));
                    }
                    actions.add(ground(template, robot, replacements));
                    for (int k = 1; k < movements.size(); k++) {
                        actions.add(ground(movements.get(k), robot, Collections.emptyMap()));
                    }
                }
            }
            for (Action action : job.getActions()) {
                actions.add(ground(action, robot, Collections.emptyMap()));
            }
            for (Action action : actions) {
                String missing = unsatisfiedPrecondition(action, state);
                if (missing != null) {
                    return action + " needs " + missing;
                }
                applyGroundEffects(action, state);
                action.setID(rebuilt.size());
                rebuilt.add(action);
            }
        }
        for (String goal : goalFacts()) {
            if (!state.contains(goal)) {
                return "the goal " + goal + " is not reached";
            }
        }
        return null;
    }

    private Action ground(Action lifted, String robot, Map<String, String> replacements) {
        Action action = problem.getAction(lifted.getName());
        for (String parameter : action.getParameters()) {
            SymbolicSymbol input = lifted.getInput(parameter);
            if (input == null) {
                continue;
            }
            String value = normalize(input.getVariable());
            if (value.equals(ROBOT_PLACEHOLDER)) {
                value = robot;
            } else {
                value = replacements.getOrDefault(value, value);
            }
            action.addInput(parameter, new SymbolicSymbol(value, problem.getType(value)));
        }
        return action;
    }

    private Action movementTemplate() {
        Action template = null;
        for (Action action : liftedPlan) {
            if (isMovement(action) && (template == null
                    || action.getParameters().size() < template.getParameters().size())) {
                template = action;
            }
        }
        return template;
    }

    private Set<String> fromFacts(Action movement) {
        Set<String> facts = new TreeSet<>();
        for (Expression<String> precondition : movement.getPreconditions()) {
            if (precondition.getConnector() == Connector.ATOM && containsRobot(precondition)
                    && locationPredicates.contains(predicateOf(precondition))) {
                facts.add(factKey(precondition, null));
            }
        }
        return facts;
    }

    private Set<String> toFacts(Action movement) {
        Set<String> facts = new TreeSet<>();
        for (Expression<String> effect : movement.getEffects()) {
            if (effect.getConnector() == Connector.ATOM && containsRobot(effect)
                    && locationPredicates.contains(predicateOf(effect))) {
                facts.add(factKey(effect, null));
            }
        }
        return facts;
    }

    private static Map<String, String> locationReplacements(Set<String> templateFacts, Set<String> actualFacts) {
        Map<String, String> replacements = new HashMap<>();
        for (String templateFact : templateFacts) {
            String[] templateTokens = tokens(templateFact);
            for (String actualFact : actualFacts) {
                String[] actualTokens = tokens(actualFact);
                if (actualTokens.length != templateTokens.length || !actualTokens[0].equals(templateTokens[0])) {
                    continue;
                }
                for (int i = 1; i < templateTokens.length; i++) {
                    if (!templateTokens[i].equals(ROBOT_PLACEHOLDER)) {
                        replacements.put(templateTokens[i], actualTokens[i]);
                    }
                }
                break;
            }
        }
        return replacements;
    }

    private Set<String> initialFacts() {
        Set<String> facts = new HashSet<>();
        for (Expression<String> fact : problem.getProblem().getInit()) {
            if (fact.getConnector() == Connector.ATOM) {
                facts.add(factKey(fact, null));
            }
        }
        return facts;
    }

    private Set<String> robotLocation(Set<String> state, String robot) {
        Set<String> location = new TreeSet<>();
        for (String fact : state) {
            String[] parts = tokens(fact);
            if (!locationPredicates.contains(parts[0])) {
                continue;
            }
            boolean found = false;
            StringBuilder builder = new StringBuilder("(").append(parts[0]);
            for (int i = 1; i < parts.length; i++) {
                if (parts[i].equals(robot)) {
                    found = true;
                    builder.append(' ').append(ROBOT_PLACEHOLDER);
                } else {
                    builder.append(' ').append(parts[i]);
                }
            }
            if (found) {
                location.add(builder.append(')').toString());
            }
        }
        return location;
    }

    private static String unsatisfiedPrecondition(Action action, Set<String> state) {
        for (Expression<String> precondition : action.getPreconditions()) {
            if (precondition.getConnector() == Connector.ATOM) {
                String fact = factKey(precondition, null);
                if (!state.contains(fact)) {
                    return fact;
                }
            } else {
                Expression<String> atom = atomOf(precondition);
                if (atom != null && state.contains(factKey(atom, null))) {
                    return "(not " + factKey(atom, null) + ")";
                }
            }
        }
        return null;
    }

    private static void applyGroundEffects(Action action, Set<String> state) {
        List<Expression<String>> effects = action.getEffects();
        for (boolean deletes : new boolean[] {true, false}) {
            for (Expression<String> effect : effects) {
                boolean negative = effect.getConnector() == Connector.NOT;
                Expression<String> atom = atomOf(effect);
                if (negative != deletes || atom == null) {
                    continue;
                }
                if (negative) {
                    state.remove(factKey(atom, null));
                } else {
                    state.add(factKey(atom, null));
                }
            }
        }
    }

    private static String[] tokens(String fact) {
        return fact.replaceAll("[()]", " ").trim().split("\\s+");
    }

    private void describeJobs() {
        for (Job job : jobs) {
            for (Action movement : job.getMovements()) {
                addRequirements(job, movement);
            }
            Set<String> madeTrue = new TreeSet<>();
            Set<String> madeFalse = new TreeSet<>();
            for (Action action : job.getActions()) {
                addRequirements(job, action);
                for (Expression<String> precondition : action.getPreconditions()) {
                    if (precondition.getConnector() != Connector.ATOM || containsRobot(precondition)) {
                        continue;
                    }
                    String fact = factKey(precondition, null);
                    if (!madeTrue.contains(fact)) {
                        job.addNeed(fact);
                    }
                }
                List<Expression<String>> effects = action.getEffects();
                for (boolean deletes : new boolean[] {true, false}) {
                    for (Expression<String> effect : effects) {
                        boolean negative = effect.getConnector() == Connector.NOT;
                        Expression<String> atom = atomOf(effect);
                        if (negative != deletes || atom == null || containsRobot(atom)) {
                            continue;
                        }
                        String fact = factKey(atom, null);
                        if (negative) {
                            if (!madeTrue.remove(fact)) {
                                madeFalse.add(fact);
                            }
                        } else {
                            madeFalse.remove(fact);
                            madeTrue.add(fact);
                        }
                    }
                }
            }
            job.setGives(madeTrue);
            job.setRemoves(madeFalse);
        }
    }

    private void addRequirements(Job job, Action action) {
        for (Expression<String> precondition : action.getPreconditions()) {
            if (precondition.getConnector() != Connector.ATOM || !containsRobot(precondition)) {
                continue;
            }
            String predicate = predicateOf(precondition);
            if (!dynamicPredicates.contains(predicate) && !locationPredicates.contains(predicate)) {
                job.addRequirement(factKey(precondition, null));
            }
        }
    }

    private void buildPrecedences() {
        Map<String, Job> lastGiver = new HashMap<>();
        Map<String, Set<Job>> readers = new HashMap<>();
        Map<String, Set<Job>> removers = new HashMap<>();
        Map<Job, Set<String>> givenInside = new HashMap<>();
        for (int i = 0; i < liftedPlan.size(); i++) {
            Job job = actionJobs.get(i);
            Action action = liftedPlan.get(i);
            if (job == null || isMovement(action)) {
                continue;
            }
            Set<String> inside = givenInside.computeIfAbsent(job, key -> new HashSet<>());
            for (Expression<String> precondition : action.getPreconditions()) {
                if (precondition.getConnector() != Connector.ATOM || containsRobot(precondition)) {
                    continue;
                }
                String fact = factKey(precondition, null);
                if (inside.contains(fact)) {
                    continue;
                }
                Job giver = lastGiver.get(fact);
                if (giver != null) {
                    addPrecedence(giver, job);
                    for (Job remover : removers.getOrDefault(fact, Collections.emptySet())) {
                        if (remover != giver) {
                            addPrecedence(remover, giver);
                        }
                    }
                }
                readers.computeIfAbsent(fact, key -> new LinkedHashSet<>()).add(job);
            }
            List<Expression<String>> effects = action.getEffects();
            for (boolean deletes : new boolean[] {true, false}) {
                for (Expression<String> effect : effects) {
                    boolean negative = effect.getConnector() == Connector.NOT;
                    Expression<String> atom = atomOf(effect);
                    if (negative != deletes || atom == null || containsRobot(atom)) {
                        continue;
                    }
                    String fact = factKey(atom, null);
                    if (negative) {
                        for (Job reader : readers.getOrDefault(fact, Collections.emptySet())) {
                            addPrecedence(reader, job);
                        }
                        readers.remove(fact);
                        if (!inside.remove(fact)) {
                            removers.computeIfAbsent(fact, key -> new LinkedHashSet<>()).add(job);
                        }
                        lastGiver.remove(fact);
                    } else {
                        lastGiver.put(fact, job);
                        inside.add(fact);
                    }
                }
            }
        }
        for (String fact : goalFacts()) {
            Job giver = lastGiver.get(fact);
            if (giver == null) {
                continue;
            }
            for (Job remover : removers.getOrDefault(fact, Collections.emptySet())) {
                if (remover != giver) {
                    addPrecedence(remover, giver);
                }
            }
        }
    }

    private static void addPrecedence(Job from, Job to) {
        to.addPredecessor(from.getId());
    }

    private Set<String> goalFacts() {
        Set<String> facts = new HashSet<>();
        Expression<String> goal = problem.getProblem().getGoal();
        if (goal == null) {
            return facts;
        }
        List<Expression<String>> atoms = new ArrayList<>();
        if (goal.getConnector() == Connector.AND) {
            atoms.addAll(goal.getChildren());
        } else {
            atoms.add(goal);
        }
        for (Expression<String> atom : atoms) {
            if (atom.getConnector() == Connector.ATOM) {
                facts.add(factKey(atom, null));
            }
        }
        return facts;
    }

    private void findPrecedenceCycle() {
        precedenceCycle.clear();
        int[] color = new int[jobs.size()];
        List<Integer> path = new ArrayList<>();
        for (Job job : jobs) {
            if (color[job.getId()] == 0 && visit(job.getId(), color, path)) {
                logger.warning("The precedences between jobs form a cycle: " + precedenceCycle
                        + ". The plan cannot be reassigned as it is.");
                return;
            }
        }
    }

    private boolean visit(int jobId, int[] color, List<Integer> path) {
        color[jobId] = 1;
        path.add(jobId);
        for (int predecessor : jobs.get(jobId).getPredecessors()) {
            if (color[predecessor] == 1) {
                precedenceCycle.addAll(path.subList(path.indexOf(predecessor), path.size()));
                return true;
            }
            if (color[predecessor] == 0 && visit(predecessor, color, path)) {
                return true;
            }
        }
        path.remove(path.size() - 1);
        color[jobId] = 2;
        return false;
    }

    private void applyEffects(Action lifted, Set<String> state, Set<String> location) {
        List<Expression<String>> effects = lifted.getEffects();
        for (boolean deletes : new boolean[] {true, false}) {
            for (Expression<String> effect : effects) {
                boolean negative = effect.getConnector() == Connector.NOT;
                Expression<String> atom = atomOf(effect);
                if (negative != deletes || atom == null || !containsRobot(atom)) {
                    continue;
                }
                String predicate = predicateOf(atom);
                Set<String> target = locationPredicates.contains(predicate) ? location
                        : dynamicPredicates.contains(predicate) ? state : null;
                if (target == null) {
                    continue;
                }
                if (negative) {
                    target.remove(factKey(atom, null));
                } else {
                    target.add(factKey(atom, null));
                }
            }
        }
    }

    private static Expression<String> atomOf(Expression<String> expression) {
        if (expression.getConnector() == Connector.ATOM) {
            return expression;
        }
        if (expression.getConnector() == Connector.NOT && !expression.getChildren().isEmpty()
                && expression.getChildren().get(0).getConnector() == Connector.ATOM) {
            return expression.getChildren().get(0);
        }
        return null;
    }

    private static String predicateOf(Expression<String> atom) {
        return normalize(atom.getSymbol().getImage());
    }

    private static boolean containsRobot(Expression<String> atom) {
        for (Symbol<String> argument : atom.getArguments()) {
            if (ROBOT_PLACEHOLDER.equals(normalize(argument.getImage()))) {
                return true;
            }
        }
        return false;
    }

    private static boolean hasArgument(Expression<String> atom, String object) {
        for (Symbol<String> argument : atom.getArguments()) {
            if (normalize(argument.getImage()).equals(object)) {
                return true;
            }
        }
        return false;
    }

    private static String factKey(Expression<String> atom, String robot) {
        StringBuilder builder = new StringBuilder("(").append(predicateOf(atom));
        for (Symbol<String> argument : atom.getArguments()) {
            String image = normalize(argument.getImage());
            builder.append(' ').append(image.equals(robot) ? ROBOT_PLACEHOLDER : image);
        }
        return builder.append(')').toString();
    }

    private static String locationName(Set<String> location) {
        List<String> names = new ArrayList<>();
        for (String fact : location) {
            List<String> parts = new ArrayList<>();
            String[] tokens = fact.replaceAll("[()]", " ").trim().split("\\s+");
            for (int i = 1; i < tokens.length; i++) {
                if (!tokens[i].equals(ROBOT_PLACEHOLDER)) {
                    parts.add(tokens[i]);
                }
            }
            names.add(String.join(" ", parts));
        }
        return String.join(", ", names);
    }

    private static String normalize(String value) {
        return value.trim().toLowerCase(Locale.ROOT);
    }
}