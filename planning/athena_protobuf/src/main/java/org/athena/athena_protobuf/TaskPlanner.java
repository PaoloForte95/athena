package org.athena.athena_protobuf;

import java.io.*;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Properties;
import java.util.Set;
import java.util.stream.Collectors;

import org.jgrapht.graph.DefaultDirectedGraph;
import org.jgrapht.graph.DefaultEdge;

import fr.uga.pddl4j.parser.Expression;

import org.pofe.athena.problems.PlanningProblem;
import org.pofe.athena.planners.Lilotane;
import org.pofe.athena.planners.MetricFF;
import org.pofe.athena.planners.TFD;
import org.pofe.athena.planners.LPG;
import org.pofe.athena.planners.AbstractPlanner;
import org.pofe.athena.parser.SymbolicSymbol;
import org.pofe.athena.plan.AbstractPlan;


class TaskPlanner{

	private static enum PLANNERS {METRICFF, LPG, TFD, LILOTANE};
	private String output_name;
	private List<String> robotDefinitions;
	private List<String> locationDefinitions;
	private String planFilename;
	private String protoFilename;
	private PlanningProblem planningProblem;


	private void readProperties() throws FileNotFoundException, IOException{
		Properties props = new Properties();

		String propertiesFilePath = "planning_params.properties";
        try (FileInputStream fis = new FileInputStream(propertiesFilePath)) {
            props.load(fis);
        }

		// Load configuration with defaults
		String robotsStr = props.getProperty("definitions.robot", "");
		robotDefinitions = Arrays.stream(robotsStr.split(","))
			.map(String::trim)
			.collect(Collectors.toList());
		String locationsStr = props.getProperty("definitions.location", "");
		locationDefinitions = Arrays.stream(locationsStr.split(","))
			.map(String::trim)
			.collect(Collectors.toList());
        this.planFilename = props.getProperty("definitions.plan_filename", "");
        this.protoFilename = props.getProperty("definitions.proto_filename", "");
        
        // Log loaded configuration
        System.out.println("=== Task Planner Configuration ===");
        System.out.println("Loaded from: " + propertiesFilePath);
        System.out.println("Robot definition: " + robotDefinitions);
        System.out.println("Location definition: " + locationDefinitions);
        System.out.println("Plan filename: " + planFilename);
        System.out.println("Proto filename: " + protoFilename);
        System.out.println("==================================\n");
	}
	

	private void computePlan(File pddlDomain, File pddlProblem, AbstractPlanner planner, ProtoExecutionPlan.Builder planPr) throws FileNotFoundException{
		planningProblem = new PlanningProblem();
		planningProblem.parse(pddlDomain, pddlProblem);
		planner.computePlan(planningProblem);

		File plan = new File(output_name);

		//Defining types 
		//Define robot, material and location types used in PDDL
		planningProblem.readPlan(plan);
		AbstractPlan executionPlan = planningProblem.getPlan();
		DefaultDirectedGraph<org.pofe.athena.parser.Action, DefaultEdge> graphPlan = executionPlan.getGraph();
		for (org.pofe.athena.parser.Action act: graphPlan.vertexSet()){
			String name = act.getName();
			Set<DefaultEdge> edges = graphPlan.incomingEdgesOf(act);
			//Create the protobuf action
			ProtoAction.Builder action = ProtoAction.newBuilder();
			action.setName(name);
			action.setId(act.getID());
			for(Expression<String> prec: act.getPreconditions()){
				action.addPreconditions(prec.toString());
			}
			for(Expression<String> effect: act.getEffects()){
				action.addEffects(effect.toString());
			}

			ArrayList<SymbolicSymbol> inputs = act.getInputs();
			

			for (SymbolicSymbol input : inputs){
				if(!Collections.disjoint(input.getType(),robotDefinitions)){
					action.setRobot(input.getVariable());			
				}
				else if (!Collections.disjoint(input.getType(),locationDefinitions)){
					action.addWaypoints(input.getVariable());			
				}
				
			}
			
			//Get the parents
			for (DefaultEdge edge : edges){
				org.pofe.athena.parser.Action parent = graphPlan.getEdgeSource(edge);
				action.addParents(parent.getID());
			}
			planPr.addAction(action);
		}

		for(org.pofe.athena.parser.Method method : executionPlan.getMethods()){
			ProtoMethod.Builder m = ProtoMethod.newBuilder();
			m.setId(method.getID());
			m.setName(method.getName());
			int parentID = -1;
			for(Integer ID : method.getActions()){
				m.addActionsIds(ID);
				for(ProtoAction protoAct: planPr.getActionList()){
					if(protoAct.getId() == ID){
						String robot = protoAct.getRobot();
						m.setRobot(robot);
						break;
					}
					
				}
				for (org.pofe.athena.parser.Action act: graphPlan.vertexSet()){
					if(act.getID() == ID){
						Set<DefaultEdge> edges = graphPlan.incomingEdgesOf(act);
						for (DefaultEdge edge : edges){
							org.pofe.athena.parser.Action parent = graphPlan.getEdgeSource(edge);
							for(org.pofe.athena.parser.Method parentMethod : executionPlan.getMethods()){
								if(parentMethod.getID() != method.getID()){
									if(parentMethod.getActions().contains(parent.getID())){
										if(parentMethod.getID() >parentID){
											parentID = parentMethod.getID();
											
										}
									}
								}
							}
						}
					}
				}
			}
			if(parentID != -1){
				m.addParents(parentID);
			}
			planPr.addMethod(m);
		}
	}

  // Main function:
  public static void main(String[] args) throws Exception {
	ProtoExecutionPlan.Builder planPr = ProtoExecutionPlan.newBuilder();
	TaskPlanner plan = new TaskPlanner();
	AbstractPlanner planner = null;
	if (args.length <= 0) {
		System.err.println("Inputs files are missing");
		System.exit(-1);
	}
  
	String ps = args[0].toUpperCase();
	switch (PLANNERS.valueOf(ps)){
		case LPG:
			planner = new LPG("src/athena/planning/athena_planner/Planners/LPG-td-1.4/");
			System.out.println("Using LPG planner!");
			plan.output_name = "plan.pddl";
			break;
		case METRICFF:
			planner = new MetricFF("src/athena/planning/athena_planner/Planners/Metric-FF-v2.1/", 0);
			System.out.println("Using MetricFF planner!");
			plan.output_name = "plan.pddl";
			break;
		case TFD:
			planner = new TFD("src/athena/planning/athena_planner/Planners/TFD/");
			System.out.println("Using TFD planner!");
			plan.output_name = "plan.hddl";
			break;
		case LILOTANE:
			planner = new Lilotane("src/athena/planning/athena_planner/Planners/Lilotane/");
			System.out.println("Using LILOTANE planner!");
			plan.output_name = "plan.hddl";
			break;
		default:
			System.out.println("Planner selected is not correct!. Available planners: " + PLANNERS.values());
			break;
	}
	File pddlDomain = new File(args[1]);
	File pddlProblem = new File(args[2]);
	plan.readProperties();
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
