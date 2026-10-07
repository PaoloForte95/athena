package org.pofe.athena.tests;


import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Set;
import java.io.*;

import org.pofe.athena.problems.PlanningProblem;
import org.pofe.athena.planners.FastDownward;
import org.pofe.athena.optimizer.Job;
import org.pofe.athena.optimizer.Optimizer;
import org.pofe.athena.optimizer.OptimizerCost;


public class OptimizationTest {



	public static void main(String[] args) throws Exception {

		File domain = new File("/home/pofe/Repositories/planning_domains/PDDL/construction/domain.pddl");
		File problem = new File("/home/pofe/Repositories/planning_domains/PDDL/construction/problems/pfile00.pddl");
		File paths = new File("/home/pofe/Repositories/planning_domains/PDDL/construction/problems/pfile00_paths.csv");
		File plan = new File("plan.pddl");

		//Create a planning problem and set the coordinator
		PlanningProblem planningProblem = new PlanningProblem();

		//Parse PDDL problem and domain files
		planningProblem.parse(domain, problem);



		FastDownward fd = new FastDownward("/home/pofe/planning_ws/src/athena/planning/athena_planner/Planners/FD/", "astar(blind())");

		fd.computePlan(planningProblem);


		OptimizerCost cost = OptimizerCost.read(paths);
		Set<String> missing = cost.getMissingLocations(planningProblem, "location");
		if (!missing.isEmpty()) {
			throw new IllegalStateException("Locations missing in " + paths + ": " + missing);
		}


		List<String> robotDefinitions = new ArrayList<>();
		robotDefinitions.add("robot");

		Optimizer optimizer = new Optimizer(robotDefinitions, Arrays.asList("drive", "transport"));
		optimizer.setCostModel(cost);

		File optimalPlanFile = plan;
		try {
			optimalPlanFile = optimizer.optimize(planningProblem, plan);
			for (Job job : optimizer.getJobs()) {
				System.out.println(job);
			}
			System.out.println(optimizer.getAssignmentResult());
		} catch (IOException e) {
			e.printStackTrace();
		}

		planningProblem.readPlan(optimalPlanFile);
	}



}