package org.pofe.athena.optimizer;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.Set;
import java.util.TreeSet;

import org.pofe.athena.parser.Action;

public class Job {

    private final int id;
    private final String robot;
    private final List<Action> movements = new ArrayList<>();
    private final List<Action> actions = new ArrayList<>();
    private final Set<String> startState;
    private Set<String> endState = new TreeSet<>();
    private String startLocation;
    private String endLocation;
    private Set<String> startLocationFacts;
    private Set<String> endLocationFacts = new TreeSet<>();
    private final Set<String> requires = new TreeSet<>();
    private final Set<String> needs = new TreeSet<>();
    private final Set<String> gives = new TreeSet<>();
    private final Set<String> removes = new TreeSet<>();
    private final Set<Integer> predecessors = new TreeSet<>();

    Job(int id, String robot, Set<String> startState) {
        this.id = id;
        this.robot = robot;
        this.startState = new TreeSet<>(startState);
    }

    void addMovement(Action movement) {
        movements.add(movement);
    }

    void addAction(Action action) {
        actions.add(action);
    }

    void setStartLocation(String startLocation, Set<String> startLocationFacts) {
        this.startLocation = startLocation;
        this.startLocationFacts = new TreeSet<>(startLocationFacts);
    }

    void close(Set<String> endState, String endLocation, Set<String> endLocationFacts) {
        this.endState = new TreeSet<>(endState);
        this.endLocation = endLocation;
        this.endLocationFacts = new TreeSet<>(endLocationFacts);
        if (startLocation == null) {
            startLocation = endLocation;
            startLocationFacts = new TreeSet<>(endLocationFacts);
        }
    }

    void addRequirement(String fact) {
        requires.add(fact);
    }

    void addNeed(String fact) {
        needs.add(fact);
    }

    void setGives(Collection<String> facts) {
        gives.clear();
        gives.addAll(facts);
    }

    void setRemoves(Collection<String> facts) {
        removes.clear();
        removes.addAll(facts);
    }

    void addPredecessor(int jobId) {
        if (jobId != id) {
            predecessors.add(jobId);
        }
    }

    public int getId() {
        return id;
    }

    public String getRobot() {
        return robot;
    }

    public List<Action> getMovements() {
        return Collections.unmodifiableList(movements);
    }

    public List<Action> getActions() {
        return Collections.unmodifiableList(actions);
    }

    public boolean hasActions() {
        return !actions.isEmpty();
    }

    public Set<String> getStartState() {
        return Collections.unmodifiableSet(startState);
    }

    public Set<String> getEndState() {
        return Collections.unmodifiableSet(endState);
    }

    public String getStartLocation() {
        return startLocation;
    }

    public String getEndLocation() {
        return endLocation;
    }

    public Set<String> getStartLocationFacts() {
        return startLocationFacts == null ? Collections.emptySet() : Collections.unmodifiableSet(startLocationFacts);
    }

    public Set<String> getEndLocationFacts() {
        return Collections.unmodifiableSet(endLocationFacts);
    }

    public Set<String> getRequires() {
        return Collections.unmodifiableSet(requires);
    }

    public Set<String> getNeeds() {
        return Collections.unmodifiableSet(needs);
    }

    public Set<String> getGives() {
        return Collections.unmodifiableSet(gives);
    }

    public Set<String> getRemoves() {
        return Collections.unmodifiableSet(removes);
    }

    public Set<Integer> getPredecessors() {
        return Collections.unmodifiableSet(predecessors);
    }

    @Override
    public String toString() {
        StringBuilder builder = new StringBuilder();
        builder.append("Job ").append(id).append(" [").append(robot).append("] ")
                .append(startLocation).append(" -> ").append(endLocation).append(":");
        for (Action movement : movements) {
            builder.append(" move").append(movement);
        }
        for (Action action : actions) {
            builder.append(' ').append(action);
        }
        builder.append("\n  state:    ").append(startState).append(" -> ").append(endState);
        builder.append("\n  requires: ").append(requires);
        builder.append("\n  needs:    ").append(needs);
        builder.append("\n  gives:    ").append(gives);
        builder.append("\n  removes:  ").append(removes);
        builder.append("\n  after:    ").append(predecessors);
        return builder.toString();
    }
}