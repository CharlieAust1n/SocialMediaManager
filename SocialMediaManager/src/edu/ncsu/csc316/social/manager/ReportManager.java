package edu.ncsu.csc316.social.manager;

import java.io.FileNotFoundException;

import edu.ncsu.csc316.dsa.list.List;
import edu.ncsu.csc316.dsa.map.Map;
import edu.ncsu.csc316.dsa.map.Map.Entry;
import edu.ncsu.csc316.social.data.Connection;
import edu.ncsu.csc316.social.data.Person;
import edu.ncsu.csc316.social.dsa.Algorithm;
import edu.ncsu.csc316.social.dsa.DSAFactory;
import edu.ncsu.csc316.social.dsa.DataStructure;

/**
 * The ReportManager class handles generating reports based on social media network data.
 * It provides methods to retrieve and organize connections by person or platform.
 * This class utilizes the SocialMediaManager to access and manage social media data.
 * 
 * @author Charlie Austin (cjausti2)
 */
public class ReportManager {
	
	/** The SocialMediaManager instance used to access social media methods/data. */
    private SocialMediaManager manager;
    /** A map of person IDS to Person objects, representing individuals in the social media network. */
    private Map<String, Person> personsMap;

    /**
     * Constructs a new ReportManager with the given input files.
     * 
     * @param peopleFile The path to the file containing information about people.
     * @param connectionFile The path to the file containing information about connections.
     * @throws FileNotFoundException If the specified file cannot be found.
     */
    public ReportManager(String peopleFile, String connectionFile) throws FileNotFoundException {
        this(peopleFile, connectionFile, DataStructure.SKIPLIST );
    }

    /**
     * Constructs a new ReportManager with the given input files and map type.
     *
     * @param peopleFile The path to the file containing information about people.
     * @param connectionFile The path to the file containing information about connections.
     * @param mapType The type of map to use for storing person data.
     * @throws FileNotFoundException If the specified file cannot be found.
     */
    public ReportManager(String peopleFile, String connectionFile, DataStructure mapType) throws FileNotFoundException {
        manager = new SocialMediaManager(peopleFile, connectionFile, mapType);
        DSAFactory.setListType(DataStructure.SINGLYLINKEDLIST);
        DSAFactory.setComparisonSorterType(Algorithm.MERGESORT);
        DSAFactory.setNonComparisonSorterType(Algorithm.RADIX_SORT);
        DSAFactory.setMapType(mapType);
        personsMap = manager.getPeople();
        
    }

    /**
     * Generates a report of connections grouped by person.
     * 
     * @return A string representing the report of connections by person.
     */
    public String getConnectionsByPerson() {
    	StringBuilder result = new StringBuilder();
    	Map<String, List<Connection>> mapOfConnectionsByPerson = manager.getConnectionsByPerson();
    	if(personsMap.isEmpty()) {
    		result.append("No people information was provided.\r\n");
    		return result.toString();
    	}
    	if(mapOfConnectionsByPerson.isEmpty()) {
    		result.append("No connections exist in the social media network.\r\n");
    		return result.toString();
    	}
    	
    	for(Entry<String, List<Connection>> entry : mapOfConnectionsByPerson.entrySet()) {
    		String userID = entry.getKey();
    		List<Connection> connections = entry.getValue();
    		String userName = personsMap.get(userID).getFirst() + " " + personsMap.get(userID).getLast();
    		result.append("Connections for ").append(userName).append(" (").append(userID).append(") {\r\n");
    		if(connections.isEmpty()) {
    			result.append("   No connections exist\r\n");
    		} else {
	    		for(Connection c : connections) {
	    			String connectionID = (userID.equals(c.getPeople()[0])) ? c.getPeople()[1] : c.getPeople()[0];
	    			String connectionName = personsMap.get(connectionID).getFirst() + " " +
	    					personsMap.get(connectionID).getLast();
	    			String date = c.getDate().toString();
	    			String platform = c.getPlatform();
	    			result.append("   ").append(connectionName).append(" (").append(connectionID).append(") on ").append(platform).append(" since ").append( 
	    					 date).append("\r\n");
	    		}
    		}
    		result.append("}\r\n");
    	}
    	return result.toString();
    }

    /**
     * Generates a report of connections grouped by platform.
     * 
     * @return A string representing the report of connections by platform.
     */
    public String getConnectionsByPlatform() {
    	 StringBuilder result = new StringBuilder();
         Map<String, List<Connection>> connectionsByPlatform = manager.getConnectionsByPlatform();
         
         if(personsMap.isEmpty()) {
     		result.append("No people information was provided.\r\n");
     		return result.toString();
     	}
         
         if(connectionsByPlatform.isEmpty()) {
     		result.append("No connections exist in the social media network.\r\n");
     		return result.toString();
     	}
         
         for(Entry<String, List<Connection>> entry : connectionsByPlatform.entrySet()) {
         	String platform = entry.getKey();
         	List<Connection> connections = entry.getValue();
         	
         	result.append("Connections on ").append(platform).append(" {\r\n");
         	for(Connection connection : connections) {
         		String timeStamp = connection.getDate().toString();
         		String[] people = connection.getPeople();
         		String user1ID = people[0];
         		String user2ID = people[1];
         		String name1 = personsMap.get(user1ID).getFirst() + " " + personsMap.get(user1ID).getLast();
         		String name2 = personsMap.get(user2ID).getFirst() + " " + personsMap.get(user2ID).getLast();
         		result.append("   ").append(timeStamp).append(": ").append(name1).append(" (").append(
         				user1ID).append(") <--> ").append(name2).append(" (").append(user2ID).append(")\r\n");
         	}
         	result.append("}\r\n");
         }
         return result.toString();
    }
}
