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




public final class LPG extends AbstractPlanner implements Callable<Integer>  {

	@Option(names = {"-o"}, description = "Specifies the file of the operators.")
	private String operator_file;
	
	@Option(names = {"-f"}, description = "Specifies the file of (init/goal) facts.")
	private String fact_file;
	
	@Option(names = {"-n"}, description = "Specifies the desired number of solutions; alternative options are -speed and -quality.")
	private String number_solution = "1";
	
	@Option(names = {"-out"}, description = "Specifies the file name for computed plans.")
	private String output_name;
	
	@Option(names = { "-h", "--help" }, usageHelp = true, description = "display a help message")
	private boolean helpRequested = false;
	
	@Option(names = {"-lpg_path"}, description = "Specifies the path for the lpg")
	private String path_lpg;
	
	@Option(names = {"-p"}, description = "specifies the path for the operator/fact files")
	private String path_files;

	
	
    /**
     * Creates a new planner with default parameters.
     */
  
    public LPG(String path) {
        super();
		this.path_lpg = path;
        logger = Logging.getLogger(LPG.class);

    }

	public int computePlan(PlanningProblem planningProblem) {
        CommandLine cmd = new CommandLine(this);
    	int exitCode = cmd.execute(
    	"-lpg_path", path_lpg,
    	"-o", planningProblem.getPlanningDomainFile(),
    	"-f", planningProblem.getPlanningProblemFile(),
    	"-n","1",
    	"-out", "plan.pddl"
    	);
    	return exitCode;
    
    }


	@Override
	public Integer call() throws Exception {
		logger.info("Computing the plan...");
		
		// Build command arguments list
		ArrayList<String> cmdArgs = new ArrayList<>();
		cmdArgs.add(path_lpg + "./lpg-td");
		cmdArgs.add("-o");
		cmdArgs.add(operator_file);
		cmdArgs.add("-f");
		cmdArgs.add(fact_file);
		cmdArgs.add("-n");
		cmdArgs.add(number_solution);
		cmdArgs.add("-out");
		cmdArgs.add(output_name);
		
		logger.info("Cmd: " + String.join(" ", cmdArgs));
		ProcessBuilder pb = new ProcessBuilder(cmdArgs);
		
		try {
			Process process = pb.start();
			BufferedReader reader = new BufferedReader(new InputStreamReader(process.getInputStream()));
			StringBuilder builder = new StringBuilder();
			String line = null;
			while ((line = reader.readLine()) != null) {
				builder.append(line);
				builder.append("\n");
			}
			toFile(builder.toString(), "plan_lpg.txt");

			if(!builder.toString().contains("Plan computed") || !builder.toString().contains("Solution number:")) {
				logger.severe("Failed to compute the plan!");
				return -1;
			}
			logger.info("Plan " + output_name + " computed!");
			ArrayList<String> plan = extractPlan(builder.toString());
			
			toFile(plan, output_name);
			return 0;
			

		} catch (IOException e) {
			logger.severe("Plan not computed!");
			e.printStackTrace();
		}
		return 0;
	}


	@Override
	protected ArrayList<String> extractPlan(String result){
		ArrayList<String> exePlan = new ArrayList<String>();
		
		// Check if plan was computed
		if (!result.contains("Plan computed:")) {
			logger.warning("Could not find plan in output");
			return exePlan;
		}
		
		// Extract the section between "Plan computed:" and "Solution number:"
		String[] parts = result.split("Plan computed:");
		if (parts.length < 2) {
			return exePlan;
		}
		
		String planSection;
		if (parts[1].contains("Solution number:")) {
			planSection = parts[1].split("Solution number:")[0].trim();
		} else {
			planSection = parts[1].trim();
		}
		
		// Split by newlines to get individual action lines
		String[] lines = planSection.split("\\n");
		
		int stepId = 0;
		for (String line : lines) {
			line = line.trim();
			
			// Look for lines that start with a time stamp like "0.0000:" or "1.0000:"
			// and contain actions in format "(ACTION-NAME ...)"
			if (line.matches("^\\d+\\.\\d+:.*") && line.contains("(") && line.contains(")")) {
				// Extract the action name between parentheses
				int startParen = line.indexOf("(");
				int endParen = line.indexOf(")", startParen);
				
				if (startParen != -1 && endParen != -1) {
					String action = line.substring(startParen + 1, endParen);
					// Skip if it's just the word "ACTION"
					if (!action.trim().equals("ACTION")) {
						exePlan.add(stepId + ": " + action);
						stepId++;
					}
				}
			}
		}
		
		return exePlan;
	}

}