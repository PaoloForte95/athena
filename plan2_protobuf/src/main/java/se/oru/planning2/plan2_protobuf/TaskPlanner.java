package se.oru.planning2.plan2_protobuf;

import java.io.*;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Set;

import org.jgrapht.graph.DefaultDirectedGraph;
import org.jgrapht.graph.DefaultEdge;

import se.planning2.plan2_protobuf.Action;
import se.planning2.plan2_protobuf.ExecutionPlan;
import se.planning2.plan2_protobuf.Method;
import se.oru.planning.planning_oru.ai_planning.problems.AbstractPlanningProblem;
import se.oru.planning.planning_oru.ai_planning.problems.NumericalPlanningProblem;
import se.oru.planning.planning_oru.ai_planning.problems.SymbolicPlanningProblem;
import se.oru.planning.planning_oru.ai_planning.problems.TemporalPlanningProblem;
import se.oru.planning.planning_oru.ai_planning.planners.AbstractPlanner;
import se.oru.planning.planning_oru.ai_planning.planners.LPG;
import se.oru.planning.planning_oru.ai_planning.planners.MetricFF;
import se.oru.planning.planning_oru.ai_planning.planners.TFD;
import se.oru.planning.planning_oru.ai_planning.planners.Lilotane;
import se.oru.planning.planning_oru.ai_planning.parser.SymbolicSymbol;
import se.oru.planning.planning_oru.ai_planning.plan.AbstractPlan.PLANTYPE;
import se.oru.planning.planning_oru.ai_planning.plan.AbstractPlan;
class TaskPlanner{

	private static enum PLANNERS {METRICFF, LPG, TFD, LILOTANE};
	private static enum PROBLEM {SYMBOLIC, NUMERICAL, TEMPORAL};
	private String robot_definition;
	private String output_name;
	private String plan_type;
	private String location_definition_;
	private String problem_type;
	private AbstractPlanningProblem planningProblem;

	private void computePlan(File pddlDomain, File pddlProblem, AbstractPlanner planner, ExecutionPlan.Builder planPr) throws FileNotFoundException{

		switch (PROBLEM.valueOf(problem_type.toUpperCase())){
				case SYMBOLIC:
					planningProblem  = new SymbolicPlanningProblem();
					break;
				case NUMERICAL:
					planningProblem  = new NumericalPlanningProblem();
					break;
				case TEMPORAL:
					planningProblem  = new TemporalPlanningProblem();
					break;
				default:
					System.out.println("Problem selected is not supported!. Supported problems: " + PROBLEM.values());
					break;
		}

		planningProblem.parse(pddlDomain, pddlProblem);
		planner.computePlan(pddlDomain.getAbsolutePath(), pddlProblem.getAbsolutePath(), output_name);

		File plan = new File(output_name);

		//Defining types 
		//Define robot, material and location types used in PDDL
		ArrayList <String> machines = new ArrayList <String>(Arrays.asList(robot_definition));
		ArrayList <String> locations = new ArrayList <String>(Arrays.asList(location_definition_));
		ArrayList <String> materials = new ArrayList <String>(Arrays.asList("material"));
		planningProblem.defineRobotType(machines);
		planningProblem.defineLocationType(locations);
		planningProblem.readPlan(plan,PLANTYPE.valueOf(plan_type));
		AbstractPlan executionPlan = planningProblem.getPlan();
		DefaultDirectedGraph<se.oru.planning.planning_oru.ai_planning.parser.Action, DefaultEdge> graphPlan = executionPlan.getGraph();
		for (se.oru.planning.planning_oru.ai_planning.parser.Action act: graphPlan.vertexSet()){
			String name = act.getName();
			Set<DefaultEdge> edges = graphPlan.incomingEdgesOf(act);
			//Create the protobuf action
			Action.Builder action = Action.newBuilder();
			action.setName(name);
			action.setId(act.getID());
			ArrayList<SymbolicSymbol> inputs = act.getInputs();
			for (SymbolicSymbol input : inputs){
				if(!Collections.disjoint(input.getType(),machines)){
					int robotID = Integer.parseInt(input.getVariable().replaceAll("[^0-9]", ""));
					action.setRobotID(robotID);			
				}
				else if (!Collections.disjoint(input.getType(),locations)){
					action.addWaypoints(input.getVariable());			
				}
				else if(!Collections.disjoint(input.getType(),materials)){
					int materialID = Integer.parseInt(input.getVariable().replaceAll("[^0-9]", ""));
					action.setMaterial(materialID);			
				}
				
			}
			
			//Get the parents
			for (DefaultEdge edge : edges){
				se.oru.planning.planning_oru.ai_planning.parser.Action parent = graphPlan.getEdgeSource(edge);
				action.addParents(parent.getID());
			}
			planPr.addAction(action);
		}

		for(se.oru.planning.planning_oru.ai_planning.parser.Method method : executionPlan.getMethods()){
			Method.Builder m = Method.newBuilder();
			m.setId(method.getID());
			m.setName(method.getName());
			for(Integer ID : method.getActions()){
				m.addActionsIds(ID);
			}
			planPr.addMethod(m);
		}
	}

  // Main function:
  public static void main(String[] args) throws Exception {
	ExecutionPlan.Builder planPr = ExecutionPlan.newBuilder();
	TaskPlanner plan = new TaskPlanner();
	AbstractPlanner planner = null;
	if (args.length <= 0) {
		System.err.println("Inputs files are missing");
		System.exit(-1);
	}
  
	String ps = args[0].toUpperCase();
	switch (PLANNERS.valueOf(ps)){
		case LPG:
			planner = new LPG("src/athena/plan2_planner/Planners/LPG-td-1.4/");
			System.out.println("Using LPG planner!");
			break;
		case METRICFF:
			planner = new MetricFF("src/athena/plan2_planner/Planners/Metric-FF-v2.1/");
			System.out.println("Using MetricFF planner!");
			break;
		case TFD:
			planner = new TFD("src/athena/plan2_planner/Planners/TFD/");
			System.out.println("Using TFD planner!");
			break;
		case LILOTANE:
			planner = new Lilotane("src/athena/plan2_planner/Planners/Lilotane/");
			System.out.println("Using LILOTANE planner!");
			break;
		default:
			System.out.println("Planner selected is not correct!. Available planners: " + PLANNERS.values());
			break;
	}
	plan.problem_type = args[1];
	File pddlDomain = new File(args[2]);
	File pddlProblem = new File(args[3]);
	plan.plan_type = args[4];
	plan.output_name = args[5];
	plan.robot_definition = args[6];
	plan.location_definition_ = args[7];

	plan.computePlan(pddlDomain,pddlProblem, planner, planPr);
	 // Write the new execution plan back to disk.
	 FileOutputStream output = new FileOutputStream("ExePlan.data");
	 try {
		planPr.build().writeTo(output);
	 } finally {
	   output.close();
	 }

  }
}
