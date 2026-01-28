package org.pofe.athena.parser;

import java.util.ArrayList;


public class SymbolicSymbol{
	
		
	public SymbolicSymbol(String variable, ArrayList<String> type) {
		this.type = type;
		this.variable = variable;
	}

	protected String variable;
	protected ArrayList<String> type;


	
	public String getVariable(){
		return this.variable;
	}
	
	public ArrayList<String> getType(){
		return this.type;
	}

	public String getInfo(){
		return "Variable: " + variable + "of type(s): " + type ;
	}
	
	@Override
	public String toString() {
		return this.variable;
		
	}


}



