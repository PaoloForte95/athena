package org.pofe.athena.tests;
import java.io.*;

import org.pofe.athena.problems.PlanningProblem;
import org.pofe.athena.planners.LPG;
import org.pofe.athena.planners.MetricFF;



public class SymbolicPlannerTest {

	public static void main(String[] args) throws Exception {
		// TODO Auto-generated method stub
		
		File domain = new File("src/athena/planning/athena_core/src/main/java/org/pofe/athena/tests/domain.pddl");
		File problem = new File("src/athena/planning/athena_core/src/main/java/org/pofe/athena/tests/p01.pddl");
		
		//Create a planning problem and set the coordinator
		PlanningProblem planningProblem = new PlanningProblem();
		int option = 0;
		
		
		//Parse PDDL problem and domain files
		planningProblem.parse(domain, problem);



		LPG lpg = new LPG("src/athena/planning/athena_planner/Planners/LPG-td-1.4/");
		MetricFF ff = new MetricFF("src/athena/planning/athena_planner/Planners/Metric-FF-v2.1/", option);


		String planner = "LPG";
		switch(planner){
			case "MetricFF":
			ff.computePlan(planningProblem);
				break;
			case "LPG":
				lpg.computePlan(planningProblem);
				break;
			default:
				break;

		}


	
	
		
		File plan = new File("plan.pddl");
	
		//Defining types 

		planningProblem.readPlan(plan);
	}

}
