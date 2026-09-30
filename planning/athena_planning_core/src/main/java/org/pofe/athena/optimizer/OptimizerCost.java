package org.pofe.athena.optimizer;

import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;

import org.pofe.athena.problems.PlanningProblem;

import fr.uga.pddl4j.parser.TypedSymbol;

public class OptimizerCost implements JobAssignment.CostModel {

    public static final String HEADER = "location";
    public static final String NO_PATH = "inf";

    private final List<String> locations;
    private final Map<String, Integer> index;
    private final double[][] lengths;
    private final String unit;

    private OptimizerCost(List<String> locations, double[][] lengths, String unit) {
        this.locations = Collections.unmodifiableList(new ArrayList<>(locations));
        this.index = new HashMap<>();
        for (int i = 0; i < locations.size(); i++) {
            index.put(locations.get(i), i);
        }
        this.lengths = lengths;
        this.unit = unit;
    }

    public static OptimizerCost read(File file) throws IOException {
        List<String> header = null;
        Map<String, double[]> rows = new HashMap<>();
        String unit = null;
        int lineNumber = 0;
        try (BufferedReader reader = Files.newBufferedReader(file.toPath(), StandardCharsets.UTF_8)) {
            String line;
            while ((line = reader.readLine()) != null) {
                lineNumber++;
                String text = line.trim();
                if (text.isEmpty()) {
                    continue;
                }
                if (text.startsWith("#")) {
                    String comment = text.substring(1).trim();
                    if (comment.toLowerCase(Locale.ROOT).startsWith("unit:")) {
                        unit = comment.substring("unit:".length()).trim();
                    }
                    continue;
                }
                String[] cells = text.split(",", -1);
                for (int i = 0; i < cells.length; i++) {
                    cells[i] = cells[i].trim();
                }
                if (header == null) {
                    header = readHeader(file, lineNumber, cells);
                } else {
                    readRow(file, lineNumber, cells, header, rows);
                }
            }
        }
        if (header == null) {
            throw new IllegalArgumentException(file + ": the file has no header line");
        }
        double[][] lengths = new double[header.size()][];
        for (int i = 0; i < header.size(); i++) {
            double[] row = rows.get(header.get(i));
            if (row == null) {
                throw new IllegalArgumentException(file + ": location " + header.get(i) + " has no row");
            }
            lengths[i] = row;
        }
        return new OptimizerCost(header, lengths, unit);
    }

    private static List<String> readHeader(File file, int lineNumber, String[] cells) {
        if (!cells[0].equalsIgnoreCase(HEADER)) {
            throw new IllegalArgumentException(file + ":" + lineNumber
                    + ": the header must start with \"" + HEADER + "\"");
        }
        if (cells.length < 2) {
            throw new IllegalArgumentException(file + ":" + lineNumber + ": the header has no locations");
        }
        List<String> names = new ArrayList<>();
        for (int i = 1; i < cells.length; i++) {
            String name = checkName(file, lineNumber, cells[i]);
            if (names.contains(name)) {
                throw new IllegalArgumentException(file + ":" + lineNumber + ": location " + name
                        + " appears twice in the header");
            }
            names.add(name);
        }
        return names;
    }

    private static void readRow(File file, int lineNumber, String[] cells, List<String> header,
                                Map<String, double[]> rows) {
        String name = checkName(file, lineNumber, cells[0]);
        int column = header.indexOf(name);
        if (column < 0) {
            throw new IllegalArgumentException(file + ":" + lineNumber + ": location " + name
                    + " is not in the header");
        }
        if (rows.containsKey(name)) {
            throw new IllegalArgumentException(file + ":" + lineNumber + ": location " + name
                    + " has more than one row");
        }
        if (cells.length - 1 != header.size()) {
            throw new IllegalArgumentException(file + ":" + lineNumber + ": expected " + header.size()
                    + " values but found " + (cells.length - 1)
                    + " (decimals must use a dot, not a comma)");
        }
        double[] row = new double[header.size()];
        for (int j = 0; j < header.size(); j++) {
            row[j] = parseValue(file, lineNumber, cells[j + 1], header.get(j));
        }
        if (row[column] != 0.0) {
            throw new IllegalArgumentException(file + ":" + lineNumber + ": the value from " + name
                    + " to itself must be 0");
        }
        rows.put(name, row);
    }

    private static double parseValue(File file, int lineNumber, String cell, String column) {
        if (cell.equalsIgnoreCase(NO_PATH)) {
            return Double.POSITIVE_INFINITY;
        }
        double value;
        try {
            value = Double.parseDouble(cell);
        } catch (NumberFormatException e) {
            throw new IllegalArgumentException(file + ":" + lineNumber + ": the value \"" + cell
                    + "\" in column " + column + " is not a number");
        }
        if (Double.isNaN(value) || value < 0.0) {
            throw new IllegalArgumentException(file + ":" + lineNumber + ": the value " + cell
                    + " in column " + column + " must be 0 or more");
        }
        return value;
    }

    private static String checkName(File file, int lineNumber, String cell) {
        String name = normalize(cell);
        if (name.isEmpty() || name.contains(" ")) {
            throw new IllegalArgumentException(file + ":" + lineNumber + ": \"" + cell
                    + "\" is not a valid location name");
        }
        return name;
    }

    public List<String> getLocations() {
        return locations;
    }

    public boolean hasLocation(String location) {
        return location != null && index.containsKey(normalize(location));
    }

    public String getUnit() {
        return unit;
    }

    public double getLength(String from, String to) {
        return lengths[indexOf(from)][indexOf(to)];
    }

    public boolean hasPath(String from, String to) {
        return !Double.isInfinite(getLength(from, to));
    }

    public Set<String> getMissingLocations(Collection<String> names) {
        Set<String> missing = new LinkedHashSet<>();
        for (String name : names) {
            if (!hasLocation(name)) {
                missing.add(normalize(name));
            }
        }
        return missing;
    }

    public Set<String> getMissingLocations(PlanningProblem problem, String locationType) {
        List<String> names = new ArrayList<>();
        for (TypedSymbol<String> object : problem.getProblem().getObjects()) {
            String name = normalize(object.getImage().toString());
            for (String type : problem.getType(name)) {
                if (normalize(type).equals(normalize(locationType))) {
                    names.add(name);
                    break;
                }
            }
        }
        return getMissingLocations(names);
    }

    @Override
    public double pathLength(String from, String to) {
        if (from == null || to == null || from.isEmpty() || to.isEmpty() || normalize(from).equals(normalize(to))) {
            return 0.0;
        }
        double length = getLength(from, to);
        if (Double.isInfinite(length)) {
            throw new IllegalArgumentException("There is no path from " + from + " to " + to);
        }
        return length;
    }

    private int indexOf(String location) {
        Integer i = location == null ? null : index.get(normalize(location));
        if (i == null) {
            throw new IllegalArgumentException("Location " + location + " is not in the path file");
        }
        return i;
    }

    private static String normalize(String value) {
        return value.trim().toLowerCase(Locale.ROOT);
    }
}