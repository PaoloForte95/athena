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
import se.oru.planning.planning_oru.ai_planning.problems.SymbolicPlanningProblem;
import se.oru.planning.planning_oru.ai_planning.planners.AbstractPlanner;
import se.oru.planning.planning_oru.ai_planning.planners.LPG;
import se.oru.planning.planning_oru.ai_planning.planners.MetricFF;
import se.oru.planning.planning_oru.ai_planning.Symbol;
import se.oru.planning.planning_oru.ai_planning.plan.ConcurrentPlan;
import se.oru.planning.planning_oru.ai_planning.plan.AbstractPlan.PLANTYPE;

class TaskPlanner{

	public static enum PLANNERS {METRICFF, LPG};

	public void computePlan(File pddlDomain, File pddlProblem, AbstractPlanner planner, ExecutionPlan.Builder planPr ) throws FileNotFoundException{

		SymbolicPlanningProblem planningProblem = new SymbolicPlanningProblem();

		planningProblem.parse(pddlDomain, pddlProblem);
		System.out.println("path" + pddlDomain.getPath());
		planner.computePlan(pddlDomain.getAbsolutePath(), pddlProblem.getAbsolutePath());

		File plan = new File("plan.pddl");

		//Defining types 
		//Define robot, material and location types used in PDDL
		ArrayList <String> machines = new ArrayList <String>(Arrays.asList("machine"));
		ArrayList <String> locations = new ArrayList <String>(Arrays.asList("location"));
		planningProblem.defineRobotType(machines);
		planningProblem.defineLocationType(locations);
		
		planningProblem.readPlan(plan,PLANTYPE.CONCURRENT);
		ConcurrentPlan executionPlan= (ConcurrentPlan) planningProblem.getPlan();
		DefaultDirectedGraph<se.oru.planning.planning_oru.ai_planning.Action, DefaultEdge> graphPlan = executionPlan.getGraph();
		int ID = 1;
		for (se.oru.planning.planning_oru.ai_planning.Action act: graphPlan.vertexSet()){
			String name = act.getName();
			Set<DefaultEdge> edges = graphPlan.incomingEdgesOf(act);
			//Create the protobuf action
			Action.Builder action = Action.newBuilder();
			action.setName(name);
			action.setId(ID);
			ArrayList<Symbol> inputs = act.getInputs();
			for (Symbol input : inputs){
				if(!Collections.disjoint(input.getType(),machines)){
					int robotID = Integer.parseInt(input.getVariable().replaceAll("[^0-9]", ""));
					action.setRobotID(robotID);			
				}
				else if (!Collections.disjoint(input.getType(),locations)){
					System.out.println("Waypoint: " + input.getVariable());
					action.addWaypoints(input.getVariable());			
				}
				
			}
			
			ID +=1;
			//Get the parents
			for (DefaultEdge edge : edges){
				se.oru.planning.planning_oru.ai_planning.Action parent = graphPlan.getEdgeSource(edge);
				action.addParents(parent.getID());
			}
			planPr.addAction(action);
		}
		
		
	}

  
	// Compute the execution Plan
	  /*static void GetPlan(AddressBook addressBook) {
	    for (Person person: addressBook.getPeopleList()) {
	      System.out.println("Person ID: " + person.getId());
	      System.out.println("  Name: " + person.getName());
	      if (!person.getEmail().isEmpty()) {
	        System.out.println("  E-mail address: " + person.getEmail());
	      }

	      for (Person.PhoneNumber phoneNumber : person.getPhonesList()) {
	        switch (phoneNumber.getType()) {
	          case MOBILE:
	            System.out.print("  Mobile phone #: ");
	            break;
	          case HOME:
	            System.out.print("  Home phone #: ");
	            break;
	          case WORK:
	            System.out.print("  Work phone #: ");
	            break;
	          default:
	            System.out.println(" Unknown phone #: ");
	            break;
	        }
	        System.out.println(phoneNumber.getNumber());
	      }
	    }
	  }
	
	// Iterates though all actions in the Execution Plan and prints them.
		  static void Printlan(AddressBook addressBook) {
		    for (Person person: addressBook.getPeopleList()) {
		      System.out.println("Person ID: " + person.getId());
		      System.out.println("  Name: " + person.getName());
		      if (!person.getEmail().isEmpty()) {
		        System.out.println("  E-mail address: " + person.getEmail());
		      }

		      for (Person.PhoneNumber phoneNumber : person.getPhonesList()) {
		        switch (phoneNumber.getType()) {
		          case MOBILE:
		            System.out.print("  Mobile phone #: ");
		            break;
		          case HOME:
		            System.out.print("  Home phone #: ");
		            break;
		          case WORK:
		            System.out.print("  Work phone #: ");
		            break;
		          default:
		            System.out.println(" Unknown phone #: ");
		            break;
		        }
		        System.out.println(phoneNumber.getNumber());
		      }
		    }
		}
	  */


  // Main function:
  public static void main(String[] args) throws Exception {
	ExecutionPlan.Builder planPr = ExecutionPlan.newBuilder();
	TaskPlanner plan = new TaskPlanner();
	AbstractPlanner planner = null;

	String ps = args[0].toUpperCase();
	switch (PLANNERS.valueOf(ps)){

		case LPG:
			planner = new LPG("/src/planning2/plan2_protobuf/Planners/LPG-td-1.4/");
			System.out.println("Using LPG planner!");
			break;
		case METRICFF:
			planner = new MetricFF("/src/planning2/plan2_protobuf/Planners/Metric-FF-v2.1/");
			System.out.println("Using MetricFF planner!");
			break;
		default:
			System.out.println("Planner selected is not correct!. Avaiable planners: METRICFF, LPG");
			break;
	}

	File pddlDomain = new File(args[1]);
	File pddlProblem = new File(args[2]);

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
