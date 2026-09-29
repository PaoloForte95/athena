/*
 * Copyright (c) 2026 by Paolo Forte <paolo.forte@oru.se>.
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
import java.io.File;
import java.io.IOException;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.concurrent.Callable;



public final class FastDownward extends AbstractPlanner implements Callable<Integer>  {

	private static final String RAW_PLAN_FILE = "plan_fd_raw.txt";

	private String configuration;

	@Option(names = {"-o"}, description = "Domain file name.")
	private String domain_file;

	@Option(names = {"-f"}, description = "Problem file name.")
	private String problem_file;

	@Option(names = {"-search"}, description = "Search configuration [preset: astar(blind())]. Ignored when -alias is set.\n"
			+ "      astar(blind())     Shortest plan, supports forall/axioms, slow on large problems\n"
			+ "      astar(lmcut())     Shortest plan, fast, does NOT support forall/axioms\n"
			+ "      lazy_greedy([ff()], preferred=[ff()])     Fast, plan not always shortest\n"
			+ ".")
	private String search = "astar(blind())";

	@Option(names = {"-alias"}, description = "Predefined configuration, for example lama-first or seq-opt-lmcut.")
	private String alias;

	@Option(names = { "-h", "--help" }, usageHelp = true, description = "usage of fast downward:")
	private boolean helpRequested = false;

	@Option(names = {"-fd_path"}, description = "Specifies the folder that contains fast-downward.py")
	private String path_fd;

	@Option(names = {"-out"}, description = "Specifies the file name for computed plan.")
	private String output_name;



	/**
	 * Creates a new Fast Downward planner.
	 *
	 * @param path the folder that contains fast-downward.py.
	 * @param configuration a search configuration such as "astar(blind())",
	 *                      or an alias such as "lama-first".
	 */
	public FastDownward(String path, String configuration) {
		super();
		this.path_fd = path;
		this.configuration = configuration;
		logger = Logging.getLogger(FastDownward.class);
	}

	public int computePlan(PlanningProblem planningProblem) {
		ArrayList<String> args = new ArrayList<>();
		args.add("-fd_path");
		args.add(this.path_fd);
		args.add("-o");
		args.add(planningProblem.getPlanningDomainFile());
		args.add("-f");
		args.add(planningProblem.getPlanningProblemFile());
		if (this.configuration != null && !this.configuration.isEmpty()) {
			args.add(this.configuration.contains("(") ? "-search" : "-alias");
			args.add(this.configuration);
		}
		args.add("-out");
		args.add("plan.pddl");

		CommandLine cmd = new CommandLine(this);
		return cmd.execute(args.toArray(new String[0]));
	}


	@Override
	public Integer call() throws Exception {
		logger.info("Computing the plan...");

		deleteOldPlanFiles();

		ArrayList<String> cmdArgs = new ArrayList<>();
		cmdArgs.add("python3");
		cmdArgs.add(new File(path_fd, "fast-downward.py").getPath());
		cmdArgs.add("--plan-file");
		cmdArgs.add(RAW_PLAN_FILE);
		if (alias != null) {
			cmdArgs.add("--alias");
			cmdArgs.add(alias);
		}
		cmdArgs.add(domain_file);
		cmdArgs.add(problem_file);
		if (alias == null) {
			cmdArgs.add("--search");
			cmdArgs.add(search);
		}

		ProcessBuilder pb = new ProcessBuilder(cmdArgs);
		pb.redirectErrorStream(true);
		try {
			Process process = pb.start();
			StringBuilder builder = new StringBuilder();
			try (BufferedReader reader = new BufferedReader(new InputStreamReader(process.getInputStream()))) {
				String line = null;
				while ((line = reader.readLine()) != null) {
					builder.append(line).append(System.lineSeparator());
				}
			}
			int fdExitCode = process.waitFor();

			toFile("plan_fd.txt", builder.toString());

			File planFile = findPlanFile();
			if (fdExitCode >= 10 || planFile == null) {
				logger.severe("Failed to compute the plan! Fast Downward exit code: " + fdExitCode
						+ ". See plan_fd.txt for details.");
				return -1;
			}

			String rawPlan = new String(Files.readAllBytes(planFile.toPath()), StandardCharsets.UTF_8);
			ArrayList<String> plan = extractPlan(rawPlan);
			logger.info("Plan " + output_name + " computed!");
			toFile(plan, output_name);
		} catch (IOException e) {
			logger.severe("Failed to compute the plan!");
			e.printStackTrace();
			return -1;
		}
		return 0;
	}

	protected ArrayList<String> extractPlan(String result) {
		ArrayList<String> exePlan = new ArrayList<String>();
		int step = 0;
		for (String line : result.split("\\R")) {
			line = line.trim();
			if (line.isEmpty() || line.startsWith(";")) {
				continue;
			}
			if (line.startsWith("(") && line.endsWith(")")) {
				line = line.substring(1, line.length() - 1).trim();
			}
			exePlan.add(step + ": " + line.toUpperCase());
			step++;
		}
		return exePlan;
	}

	private File findPlanFile() {
		File plan = new File(RAW_PLAN_FILE);
		if (plan.exists()) {
			return plan;
		}
		File lastPlan = null;
		int i = 1;
		while (new File(RAW_PLAN_FILE + "." + i).exists()) {
			lastPlan = new File(RAW_PLAN_FILE + "." + i);
			i++;
		}
		return lastPlan;
	}

	private void deleteOldPlanFiles() {
		new File(RAW_PLAN_FILE).delete();
		int i = 1;
		while (new File(RAW_PLAN_FILE + "." + i).delete()) {
			i++;
		}
	}

}
