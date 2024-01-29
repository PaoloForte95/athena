package org.athena.athena_protobuf;

import java.io.FileNotFoundException;
import java.io.FileReader;

import javax.json.Json;
import javax.json.JsonArray;
import javax.json.JsonObject;
import javax.json.JsonReader;

import java.util.SortedMap;
import java.util.TreeMap;
import java.util.logging.ConsoleHandler;
import java.util.logging.Formatter;
import java.util.logging.Handler;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.logging.LogRecord;

public class ProjectReader {

    private static Logger logger = Logger.getLogger(ProjectReader.class.getSimpleName());
    private String domain;
    private String plan;
    private SortedMap<Integer,Double> mats; 


    public ProjectReader(){
        mats = new TreeMap<>();
    }
    

    private void configureLogger(){
        for(Handler h : logger.getParent().getHandlers()) { logger.getParent().removeHandler(h); }
        ConsoleHandler h = new ConsoleHandler();
        h.setFormatter(getFormatter(logger));
        h.setLevel(Level.ALL);
        logger.addHandler(h);
    }

    private static Formatter getFormatter(final Logger logger) {
		return new Formatter() {
			@Override
			public String format(LogRecord arg0) {
				String prefix = "";
				String ret = prefix + ("[" + logger.getName() + "] " + arg0.getMessage() + "\n");
				return ret;
			}
		};
    }

    public String getPlan(){
        return this.plan;
    }

    public String getDomain(){
        return this.domain;
    }

    public SortedMap<Integer,Double> getMats(){
        return this.mats;
    }

    public void readProject(String jsonFilePath) {
        JsonReader reader;
        configureLogger();
        try {
            reader = Json.createReader(new FileReader(jsonFilePath));
            // Read JSON file into JsonObject
            JsonObject jsonObject = reader.readObject();

            // Access data from JsonObject
            domain = jsonObject.getString("AiddlDomain");
            plan = jsonObject.getString("AiddlProblem");
            logger.info("AiddlDomain: " + domain);
            logger.info("AiddlProblem: " + plan);
            // Access materials array
            JsonArray materialsArray = jsonObject.getJsonArray("materials");
            for (int i = 0; i < materialsArray.size(); i++) {
                JsonObject material = materialsArray.getJsonObject(i);

                String ID = material.getString("ID");
                int amount = material.getInt("amount");
                String location = material.getString("location");
                String stacked = material.getString("stacked");
                mats.put(Integer.valueOf(ID), Double.valueOf(amount));
                logger.info("Material ID: " + ID);
                logger.info("Amount: " + amount);
                logger.info("Location: " + location);
                logger.info("Stacked: " + (!stacked.isEmpty()));
                System.out.println("--------------------");
            }
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
    }
    
    
}
