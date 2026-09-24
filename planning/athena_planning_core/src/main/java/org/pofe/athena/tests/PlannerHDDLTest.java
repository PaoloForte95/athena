package org.pofe.athena.tests;
import java.io.*;

import org.pofe.athena.problems.PlanningProblem;
import org.pofe.athena.planners.TFD;
import org.pofe.athena.planners.Lilotane;


public class PlannerHDDLTest {

	public static void main(String[] args) throws Exception {
		
		File domain = new File("/home/pofe/Repositories/planning_domains/HDDL/construction/domains/domain.hddl");
		File problem = new File("/home/pofe/Repositories/planning_domains/HDDL/construction/problems/pfile02.hddl");
		
		TFD tfd = new TFD("src/athena/planning/athena_planner/Planners/TFD/");
		Lilotane lil = new Lilotane("src/athena/planning/athena_planner/Planners/Lilotane/");
		
		String planner = "TFD";
		int runs = 30;
		double[] times = new double[runs];
		double[] lengths = new double[runs];

		File results = new File("/home/pofe/ICRA27/basic_planner_results_stacking.txt");
		PrintWriter writer = new PrintWriter(new FileWriter(results));
		writer.printf("planner\trun\tplan_length\ttime_ms%n");

		for (int i = 0; i < runs; i++) {
			PlanningProblem planningProblem = new PlanningProblem();
			planningProblem.parse(domain, problem);

			long start = System.nanoTime();
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
			long elapsed = System.nanoTime() - start;
			times[i] = (elapsed / 1_000_000.0);
			planningProblem.readPlan(plan);
			lengths[i] = planningProblem.getPlan().makespan();

			writer.printf("%s\t%d\t%.2f\t%.3f%n", planner, i + 1, lengths[i], times[i]);
			writer.flush();
			System.out.printf("Run %d, planner: %s, plan length: %.2f, computation time: %.3f ms%n",
				i + 1, planner, lengths[i], times[i]);
		}

		double sum = 0.0;
		for (double t : times) {
			sum += t;
		}
		double mean = sum / runs;

		double sqSum = 0.0;
		for (double t : times) {
			sqSum += (t - mean) * (t - mean);
		}
		double std = Math.sqrt(sqSum / (runs - 1));

		double[] sorted = times.clone();
		java.util.Arrays.sort(sorted);
		double min = sorted[0];
		double max = sorted[runs - 1];
		double median = runs % 2 == 0
			? (sorted[runs / 2 - 1] + sorted[runs / 2]) / 2.0
			: sorted[runs / 2];

		double lengthSum = 0.0;
		for (double l : lengths) {
			lengthSum += l;
		}
		double meanLength = lengthSum / runs;

		writer.printf("%nPlanner: %s over %d runs%n", planner, runs);
		writer.printf("Mean time: %.3f ms, Std: %.3f ms%n", mean, std);
		writer.printf("Median time: %.3f ms, Min: %.3f ms, Max: %.3f ms%n", median, min, max);
		writer.printf("Mean plan length: %.2f%n", meanLength);
		writer.close();

	}


}