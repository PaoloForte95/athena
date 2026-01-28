package org.pofe.athena.tests;
import java.io.*;

import org.pofe.athena.problems.PlanningProblem;
import org.pofe.athena.planners.TFD;
import org.pofe.athena.planners.Lilotane;


public class PlannerHDDLTest {

	public static void main(String[] args) throws Exception {
		
		File domain = new File("src/athena/planning/athena_core/src/main/java/org/pofe/athena/tests/domain.hddl");
		File problem = new File("src/athena/planning/athena_core/src/main/java/org/pofe/athena/tests/pfile01.hddl");
		
		
		TFD tfd = new TFD("src/athena/planning/athena_planner/Planners/TFD/");
		Lilotane lil = new Lilotane("src/athena/planning/athena_planner/Planners/Lilotane/");
		
		PlanningProblem planningProblem = new PlanningProblem();
		
		//Parse PDDL problem and domain files
		planningProblem.parse(domain, problem);

		String planner = "TFD";
		switch(planner){
			case "TFD":
				tfd.computePlan(planningProblem);
				break;
			case "Lilotane":
				lil.computePlan(planningProblem);
				break;
			default:
				break;

		}

		File plan = new File("plan.hddl");

		planningProblem.readPlan(plan);
	}

}
