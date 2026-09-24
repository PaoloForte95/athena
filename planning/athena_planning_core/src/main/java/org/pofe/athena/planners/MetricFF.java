/*
 * Copyright (c) 2023 by Paolo Forte <paolo.forte@oru.se>.
 *
 * This file is part of planning_oru library.
 *
 * planning_oru is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * planning_oru is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with PDDL4J.  If not, see <http://www.gnu.org/licenses/>
 */

package org.pofe.athena.planners;

import org.pofe.athena.problems.PlanningProblem;
import org.pofe.athena.util.Logging;

import picocli.CommandLine;
import picocli.CommandLine.Option;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.concurrent.Callable;



public final class MetricFF extends AbstractPlanner implements Callable<Integer>  {

	private int option;

	@Option(names = {"-o"}, description = "Operator file name.")
	private String operator_file;
	
	@Option(names = {"-f"}, description = "Fact file name.")
	private String fact_file;
	
	
	@Option(names = {"-s"}, description = "Search configuration [preset: s=5]; '+H': helpful actions pruning\n"
			+ "      0     Standard-FF: EHC+H then BFS (cost minimization: NO)\n"
			+ "      1     BFS (cost minimization: NO)\n"
			+ "      2     BFS+H (cost minimization: NO)\n"
			+ "      3     Weighted A* (cost minimization: YES)\n"
			+ "      4     A*epsilon (cost minimization: YES)\n"
			+ "      5     EHC+H then A*epsilon (cost minimization: YES)\n"
			+ ".")
	private String search_configuration = "0";
	
	@Option(names = { "-h", "--help" }, usageHelp = true, description = "usage of ff:")
	private boolean helpRequested = false;
	
	@Option(names = {"-w"}, description = "Set weight w for search configs 3,4,5 [preset: w=5]")
	private String weight = "5";
	
	@Option(names = {"-C"}, description = "Do NOT use cost-minimizing relaxed plans for options 3,4,5")
	private boolean avoid_cost_minimizing = false;
	
	@Option(names = {"-b"}, description = "Fixed upper bound on solution cost (prune based on g+hmax); active only with cost minimization")
	private String upper_bound_cost;
	
	@Option(names = {"-p"}, description = "Path for operator and fact file")
	private String path_files;
	
	@Option(names = {"-metric_ff_path"}, description = "Specifies the path for the metric ff planner")
	private String path_metric_ff;
	
	@Option(names = {"-out"}, description = "Specifies the file name for computed plan.")
	private String output_name;

	
	
    /**
     * Creates a new planner with default parameters.
     */
  
    public MetricFF(String path, int option) {
        super();
		this.path_metric_ff = path;
        logger = Logging.getLogger(MetricFF.class);
		this.option = option;

    }

    public int computePlan(PlanningProblem planningProblem) {
        CommandLine cmd = new CommandLine(this);
    	int exitCode = cmd.execute(
    	"-metric_ff_path", this.path_metric_ff,
    	"-o", planningProblem.getPlanningDomainFile(),
    	"-f", planningProblem.getPlanningProblemFile(),
		"-s", Integer.toString(this.option),
    	"-out", "plan.pddl"
		
    	);
    	return exitCode;
    }
    
    
	@Override
	public Integer call() throws Exception {
		logger.info("Computing the plan...");
		
		// Build command arguments list similar to computePlan
		ArrayList<String> cmdArgs = new ArrayList<>();
		cmdArgs.add(path_metric_ff + "ff");
		cmdArgs.add("-o");
		cmdArgs.add(operator_file);
		cmdArgs.add("-f");
		cmdArgs.add(fact_file);
		cmdArgs.add("-s");
		cmdArgs.add(search_configuration);
		cmdArgs.add("-w");
		cmdArgs.add(weight);
		
		if (avoid_cost_minimizing) {
			cmdArgs.add("-C");
		}
		
		if (upper_bound_cost != null) {
			cmdArgs.add("-b");
			cmdArgs.add(upper_bound_cost);
		}
		
		ProcessBuilder pb = new ProcessBuilder(cmdArgs);
		try {
			Process process = pb.start();
			BufferedReader reader = new BufferedReader(new InputStreamReader(process.getInputStream()));
			StringBuilder builder = new StringBuilder();
			String line = null;
			while ((line = reader.readLine()) != null) {
				builder.append(line);
			}
			
			toFile("plan_metric_ff.txt", builder.toString());

			if (!builder.toString().contains("step")) {
				logger.severe("Failed to compute the plan!");
				return -1;
    		}

			ArrayList<String> plan = extractPlan(builder.toString());
			logger.info("Plan " + output_name + " computed!");
			toFile(plan, output_name);
		} catch (IOException e) {
			logger.severe("Failed to compute the plan!");
			e.printStackTrace();
		}
		return 0;
	}

	protected ArrayList<String> extractPlan(String result){
    ArrayList<String> exePlan = new ArrayList<String>();
    
    
    String[] output = result.split("step");
    if (output.length < 2) {
        return exePlan;
    }
    
    // Try to split by "time spent:" first, then "plan cost:" if not found
    String planSection;
    if (output[1].contains("time spent:")) {
        String[] output_plan = output[1].split("time spent:");
        planSection = output_plan[0].trim();
    } else if (output[1].contains("plan cost:")) {
        String[] output_plan = output[1].split("plan cost:");
        planSection = output_plan[0].trim();
    } else {
        logger.warning("Could not find end marker (time spent or plan cost) in output");
        return exePlan;
    }
    
    // Split by pattern: whitespace followed by digit and colon
    String[] actions = planSection.split("\\s+(?=\\d+:)");
    
    for (String action : actions) {
        action = action.trim();
        if (action.isEmpty() || !action.contains(":")) {
            continue;
        }
        // Keep the full "ID: ACTION" format
        exePlan.add(action);
    }
    
    return exePlan;
}
	


}
