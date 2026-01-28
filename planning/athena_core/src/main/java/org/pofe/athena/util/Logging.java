package org.pofe.athena.util;

import java.io.File;
import java.io.IOException;
import java.io.Serializable;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.Calendar;
import java.util.HashMap;
import java.util.Map.Entry;
import java.util.logging.ConsoleHandler;
import java.util.logging.FileHandler;
import java.util.logging.Formatter;
import java.util.logging.Handler;
import java.util.logging.Level;
import java.util.logging.LogRecord;
import java.util.logging.Logger;

/**
 * Utility class to provide logging facilities 
 * @author Paolo Forte
 */

public final class Logging implements Serializable{

	private transient static HashMap<Class<?>,Logger> loggers = new HashMap<Class<?>,Logger>();

	private transient static HashMap<Class<?>,Level> tempLevels = new HashMap<Class<?>,Level>();

	private static final long serialVersionUID = 7526472295622776139L;

	private static Level globalLevel = null;
	
	private static String logDir = null;

	static { }

	/**
	 * Instruct loggers to log all output to a directory. Each class will be logged to a file named
	 * <code>className.log</code>.
	 * @param dir The directory to log to.
	 */
	public static void setLogDir(String dir) {
		try { 
			logDir = dir;
			for (Entry<Class<?>, Logger> en : loggers.entrySet()) {
				FileHandler fh = new FileHandler(logDir+File.separator+en.getKey().getSimpleName()+".log");
				fh.setFormatter(getFormatter(en.getValue(),true));
				en.getValue().addHandler(fh);
			}
		}
		catch (SecurityException e) { e.printStackTrace(); }
		catch (IOException e) { e.printStackTrace(); }  
	}

	private Logging() {}

	/**
	 * Set a desired log level for all loggers.
	 * @param l The desired log level.
	 */
	public static void setLevel(Level l) {
		for (Logger log : loggers.values()) log.setLevel(l);
		globalLevel = l;
	}

	/**
	 * Set a desired log level for a specific class.
	 * @param c The class to set the log level for.
	 * @param l The desired log level.
	 */
	public static void setLevel(Class<?> c, Level l) /* throws LoggerNotDefined */ {
		if (!loggers.containsKey(c)) tempLevels.put(c, l);
		else loggers.get(c).setLevel(l);
		//System.out.println("Set level " + l + " for logger " + loggers.get(c).getName());
	}

	private static Formatter getFormatter(final Logger logger, final boolean timeStamp) {
		return new Formatter() {
			@Override
			public String format(LogRecord arg0) {
				String prefix = "";
				if (timeStamp) prefix = Calendar.getInstance().getTimeInMillis()+"@";
				String ret = prefix + ("[" + logger.getName() + "] " + arg0.getMessage() + "\n");
				return ret;
			}
		};
	}
	/**
	 * Provides a reference to a {@link Logger}.
	 * @param c The class within which the logger should be used.
	 * @return A {@link Logger} that can be used to log messages in the given class.
	 */
	public static Logger getLogger(final Class<?> c) {
		if (loggers.get(c) == null) {
			//System.out.println("Making new logger for " + c.getSimpleName());
			final Logger logger = Logger.getLogger(c.getSimpleName());
			loggers.put(c, logger);
			for(Handler h : logger.getParent().getHandlers()) { logger.getParent().removeHandler(h); }
			ConsoleHandler h = new ConsoleHandler();
			h.setFormatter(getFormatter(logger,false));
			h.setLevel(Level.ALL);
			logger.addHandler(h);
			if (tempLevels.keySet().contains(c)) {
				logger.setLevel(tempLevels.get(c));
				tempLevels.remove(c);
			}
			if (logDir != null) {
				try {
					FileHandler fh = new FileHandler(logDir+File.separator+c.getSimpleName()+".log");
					fh.setFormatter(getFormatter(logger,true));
					logger.addHandler(fh);
				}
				catch (SecurityException e) { e.printStackTrace(); }
				catch (IOException e) { e.printStackTrace(); }
			}
			if (globalLevel != null) logger.setLevel(globalLevel);
			return logger;
		}
		//System.out.println("Returning old logger for " + c.getSimpleName());
		return loggers.get(c);
	}

	public static String printDouble(double d, int precision) {
		String fmt = "#0."; 
		for (int i = 0; i < precision; i++) fmt += "0"; 
		NumberFormat formatter = new DecimalFormat(fmt); 
		return formatter.format(d);	
	}
	

}