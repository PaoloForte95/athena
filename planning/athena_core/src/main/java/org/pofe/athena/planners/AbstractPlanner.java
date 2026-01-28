package org.pofe.athena.planners;
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
 * along with planning_oru.  If not, see <http://www.gnu.org/licenses/>
 */

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.logging.Logger;


import org.pofe.athena.problems.PlanningProblem;

public abstract class AbstractPlanner {

    /**
     * The class logger.
     */
	protected Logger logger;



    /**
     * Compute the plan using the desired planner.
     */
    public abstract int computePlan(PlanningProblem planningProblem);



    /**
     * Save the plan into a file.
     * @param String the list of actions
     */
	protected void toFile(ArrayList<String> plan, String fileName) {
	    try {
	        BufferedWriter bw = new BufferedWriter(new FileWriter((fileName),false));
	        for (String action: plan) {
	        	bw.write(action);
	            bw.newLine();
	        }
	        bw.flush();
	        bw.close();
	    } catch (IOException e) {}
	}

    protected void toFile(String fileName, String content) {
        try {
            BufferedWriter bw = new BufferedWriter(new FileWriter((fileName),false));
            bw.write(content);
            bw.newLine();
            bw.flush();
            bw.close();
        } catch (IOException e) {}
    }

    /**
    * Extract a plan from the result of the planner.
     * @param result the String result of the planner 
     * @return The set of actions contained in the execution plan
     */
    protected abstract ArrayList<String> extractPlan(String result);

}
