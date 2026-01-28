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

public class Lilotane  extends AbstractPlanner implements Callable<Integer>{
    @Option(names = {"-o"}, description = "Specifies the file of the operators.")
     private String operator_file;
     
     @Option(names = {"-f"}, description = "Specifies the file of (init/goal) facts.")
     private String fact_file;
     
     @Option(names = {"-t"}, description = "Try finding an initial plan for up to #secs (without optimization: total allowed runtime; 0: no limit).")
     private String time;
     
     @Option(names = { "-h", "--help" }, usageHelp = true, description = "display a help message")
     private boolean helpRequested = false;

     @Option(names = {"-p"}, description = "specifies the path for the operator/fact files")
     private String path_files;

     @Option(names = {"-planner_path"}, description = "Specifies the path for the planner")
     private String path_planner;

     @Option(names = {"-out"}, description = "Specifies the file name for computed plans.")
     private String output_name;
 
     
     
     /**
      * Creates a new planner with default parameters.
      */
   
     public Lilotane(String path) {
         super();
         this.path_planner = path;
         logger = Logging.getLogger(Lilotane.class);
    
     }
     
     public int computePlan(PlanningProblem planningProblem) {
        CommandLine cmd = new CommandLine(this);
        int exitCode = cmd.execute(
        "-planner_path", path_planner,
        "-o", planningProblem.getPlanningDomainFile(),
        "-f", planningProblem.getPlanningProblemFile(),
        "-out", "plan.hddl"
        );
        return exitCode;
     }

 
     @Override
     public Integer call() throws Exception {
         logger.info("Computing the plan...");
         
         // Build command arguments list
         ArrayList<String> cmdArgs = new ArrayList<>();
         cmdArgs.add(path_planner + "./lilotane");
         cmdArgs.add(operator_file);
         cmdArgs.add(fact_file);
         
         String error_msg = "";
         ProcessBuilder pb = new ProcessBuilder(cmdArgs);
         logger.info("Command: " + String.join(" ", cmdArgs));
         
         try {
             Process process = pb.start();
             BufferedReader reader = new BufferedReader(new InputStreamReader(process.getInputStream()));
             StringBuilder builder = new StringBuilder();
             String line = null;
             while ((line = reader.readLine()) != null) {
                if(!line.isEmpty() && !line.isBlank()){
                    builder.append(line);
                    builder.append("\n");
                }
                if(line.contains("error")){
                    error_msg = line;
                    logger.info("Error while computing the plan! Error: " + error_msg);
                    return -1;
                }
             }
             
            logger.info("Plan " + output_name + " computed!");
            toFile("plan_lilotane.txt", builder.toString());
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
         
         // Check if plan markers exist
         if (!result.contains("==>") || !result.contains("<==")) {
             logger.warning("Could not find plan markers in output");
             return exePlan;
         }
     
         String plan = result.substring(result.indexOf("==>") + 4, result.indexOf("<==") - 10);
         
         for(String action : plan.split("\n")){
            action = action.trim();
            if(!action.isEmpty() && !action.isBlank()){
                exePlan.add(action);
            }
         }
         return exePlan;
 
     }
	
}