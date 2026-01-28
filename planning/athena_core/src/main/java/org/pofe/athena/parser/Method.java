package org.pofe.athena.parser;

import java.util.ArrayList;
import java.util.List;


import fr.uga.pddl4j.parser.Expression;

public class Method {

    private String name;
    private String task;
	private int ID;
	private List <Expression<String>> preconditions;
    private List<Integer> subtasks;

    public Method(){
        this("", 1, "");
    }

    public Method(int ID){
        this("", ID, "");
    }

	/**
	 * Create an action with a specific name and a specific ID.
	 * @param name -> the action name
	 * @param ID -> the action ID
	 */
	
	 public Method(String name,int ID, String task) {
		this.name = name;
		this.ID = ID;
        this.task = task;
        this.preconditions =  new ArrayList <>();
        subtasks = new ArrayList<>();
	}


	/**
	 * Set the ID of the method 
	 * @param ID -> the ID to associate to the method
	 */
	public void setID(int ID) {
		this.ID = ID;
	}

    public int getID(){
        return ID;
    }

    public void setName(String name){
        this.name = name;
    }

    public String getName(){
        return name;
    }

    public String getTask(){
        return task;
    }

    public void addSubtask(int ID){
        if(!subtasks.contains(ID)){
            subtasks.add(ID);
        }
    }

    public void addSubtask(List<Integer> IDs){
        for(Integer ID: IDs){
            addSubtask(ID);
        }
    }

    public void addPrecondition(Expression<String> precondition){
        if(!preconditions.contains(precondition)){
            preconditions.add(precondition);
        }
    }


    public  List<Integer> getActions(){
        return subtasks;
    }
    
    
}
