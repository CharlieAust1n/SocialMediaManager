package edu.ncsu.csc316.social.manager;

import java.io.FileNotFoundException;
import java.util.Comparator;

import edu.ncsu.csc316.dsa.list.List;
import edu.ncsu.csc316.dsa.map.Map;
import edu.ncsu.csc316.dsa.sorter.Sorter;
import edu.ncsu.csc316.social.data.Connection;
import edu.ncsu.csc316.social.data.Person;
import edu.ncsu.csc316.social.dsa.Algorithm;
import edu.ncsu.csc316.social.dsa.DSAFactory;
import edu.ncsu.csc316.social.dsa.DataStructure;
import edu.ncsu.csc316.social.io.InputReader;

/**
 * The SocialMediaManager class handles storing information from files into maps.
 * It provides methods to retrieve and manage data related to people and their connections.
 * This class utilizes various data structures and sorting algorithms to efficiently manage the data.
 * 
 * @author Charlie Austin (cjausti2)
 */
public class SocialMediaManager {
	
	/** List of people that will be read in from a file. */
	private List<Person> listOfPersons;
	/** List of connections that will be read in from a file. */
	private List<Connection> listOfConnections;
	/** A map of person IDS to Person objects, representing individuals in the social media network. */
	private Map<String, Person> mapOfPersons;
	/** Sorter used to sort connections. */
	private Sorter<Connection> sorter;
	/** Sorter to be used for sorting a single connection. */
	private Sorter<String> nameSorter;

	
	/**
	 * Constructs a new SocailMediaManager with the given input files. The map is defaulted to a SkipListMap.
	 * 
	 * @param peopleFile The path to the file containing information about people.
	 * @param connectionFile The path to the file containing information about connections.
	 * @throws FileNotFoundException If the specified file cannot be found.
	 */
    public SocialMediaManager(String peopleFile, String connectionFile) throws FileNotFoundException {
        this(peopleFile, connectionFile, DataStructure.SKIPLIST);
    }
    
    /**
     * Constructs a new SocialMediaManager with the given input files and given map type.
     * 
     * @param peopleFile The path to the file containing information about people.
     * @param connectionFile The path to the file containing information about connections.
     * @param mapType The type of map to use for storing person data
     * @throws FileNotFoundException If the specified file cannot be found.
     */
    public SocialMediaManager(String peopleFile, String connectionFile, DataStructure mapType)
            throws FileNotFoundException {
        DSAFactory.setListType(DataStructure.ARRAYBASEDLIST);
        DSAFactory.setComparisonSorterType(Algorithm.MERGESORT);
        DSAFactory.setNonComparisonSorterType(Algorithm.RADIX_SORT);
        DSAFactory.setMapType(mapType);
        
        this.listOfPersons = InputReader.readPersonData(peopleFile); // Reading in the list of people.
        this.listOfConnections = InputReader.readConnectionData(connectionFile); // Reading in the list of connections.
        mapOfPersons = DSAFactory.getMap(null); // Creating a mapOfPersons.
        getPeople(); // Populate the mapOfPersons.
        
    }
    
    /**
     * Retrieves a map of entries with String user_name keys and Person values. 
     * The map utilizes the natural order (key1 is compared against key2).
     * @return The map of entries with String user_name keys and Person values. 
     */
    public Map<String, Person> getPeople() {
    	for(Person person : listOfPersons) {
    		mapOfPersons.put(person.getId(), person);
    	}
    	return mapOfPersons;
    }
    
    /**
     * Retrieves a map of connections grouped by person. The list of connections for each person
     * is sorted by name. If the names are the same then by ID. If the IDs are the same, then by date of connection.
     * 
     * @return A map of connections grouped by person
     */
    public Map<String, List<Connection>> getConnectionsByPerson() {
    	// Initialize a new map to store connections by person. Key is the userID. Value is the list of connections.
    	// Keys are sorted by natural order.
    	Map<String, List<Connection>> connectionsByPersonMap = DSAFactory.getMap(null);
    	
    	// Check to see if the list of connections is empty to avoid null pointer errors.
    	if(listOfConnections.isEmpty()) {
    		return connectionsByPersonMap;
    	}
    	
    	// Check to see if the map of persons is empty to avoid null pointer error.
    	if(mapOfPersons.isEmpty()) {
    		return connectionsByPersonMap;
    	}
    	   
    	// Iterate through mapOfPersons and add userIDs as keys and an empty list as values.
    	for(String userID : mapOfPersons) {
    		connectionsByPersonMap.put(userID, DSAFactory.getIndexedList());
    	}
    	
    	// Iterate through the listOfConnections and add connection to the userID key as its value.
    	for(Connection connection : listOfConnections) {
    		String[] people = connection.getPeople();
    		connectionsByPersonMap.get(people[0]).addLast(connection);
    		connectionsByPersonMap.get(people[1]).addLast(connection);
    	}
    	
    	// Iterate through the connectionsByPersonMap.
    	for(Map.Entry<String, List<Connection>> entry : connectionsByPersonMap.entrySet()) {
    		String userID = entry.getKey();
    		List<Connection> unsortedConnections = entry.getValue();
    		
    		// Converting each value to an array.
    		Connection[] connectionArray = new Connection[unsortedConnections.size()];
    		int idx = 0;
    		for(Connection connection : unsortedConnections) {
    			connectionArray[idx++] = connection;
    		}
    		// Sort the array using the customComparatorName
    		sorter = DSAFactory.getComparisonSorter(new CustomComparatorName(userID));
    		sorter.sort(connectionArray);
    		
    		// Convert the array back to a list.
    		List<Connection> sortedList = DSAFactory.getIndexedList();
    		for(Connection connection : connectionArray) {
    			sortedList.addLast(connection);
    		}
    		
    		// Replace the value as the sorted list.
    		connectionsByPersonMap.put(userID, sortedList);
    	}
    	
    	return connectionsByPersonMap;
    }
    
    /**
     * Retrieves a map of connections grouped by platform. Each list of connection is sorted
     * by date. If the dates are the same, then they are sorted by the connection ID.
     * 
     * For the individual connection, the order of people is sorted by their names. If the names are
     * the same, then it is sorted by ID.
     * 
     * @return A map of connections grouped by platform
     */
    public Map<String, List<Connection>> getConnectionsByPlatform() {
    	// Initialize a new map to store connections by platform. Key is the platform. Value is the list of connections.
    	// Keys are sorted by natural order.
    	Map<String, List<Connection>> connectionsByPlatformMap = DSAFactory.getMap(null);
    	
    	// Check if the list of connections is empty to avoid null pointer errors.
    	if(listOfConnections.isEmpty()) {
    		return connectionsByPlatformMap;
    	}
    	
    	// Iterate through the list of connections.
    	for(Connection connection : listOfConnections) {
    		// Retrieve the platform the current connection.
    		String platform = connection.getPlatform();
    		
    		// Retrieve the list of connections linked to the platform.
    		List<Connection> connectionsForPlatform = connectionsByPlatformMap.get(platform);
    		
    		// Check to see if the list exists.
    		if(connectionsForPlatform == null) {
    			// List does not exist so create one.
    			connectionsForPlatform = DSAFactory.getIndexedList();
    			// Insert the list into the output map.
    			connectionsByPlatformMap.put(platform, connectionsForPlatform);
    		}
    		// Add the current connection to the list of connections for the platform.
    		connectionsForPlatform.addLast(connection);	
    	}
    	
    	for(List<Connection> connectionsForPlatform : connectionsByPlatformMap.values()) {
    		// Convert the list to an array.
    		Connection[] connectionsArray = new Connection[connectionsForPlatform.size()];
    		int idx = 0;
    		// Store the connections into the array.
    		for(Connection c : connectionsForPlatform) {
    			connectionsArray[idx++] = c;
    		}
    		sorter = DSAFactory.getComparisonSorter(new CustomComparatorDate()); // Create the sorter.
    		// Sort the array of connections.
    		sorter.sort(connectionsArray);
    		
    		while(!connectionsForPlatform.isEmpty()) {
    			connectionsForPlatform.removeLast();
    		}
    		
    		// Sort the connection itself.
    		nameSorter = DSAFactory.getComparisonSorter(new NameComparator());
    		for(Connection c : connectionsArray) {
    			String[] people = c.getPeople();
    			nameSorter.sort(people);
    			connectionsForPlatform.addLast(c);
    		}
    	}
    	
    	return connectionsByPlatformMap;
    }
    
    /**
     * Private class to compare two connections by name, then userIDS, then date of connection.
     */
    private class CustomComparatorName implements Comparator<Connection> {
    	
    	/** A field to keep track of the userID that contains the connection. */
    	private String userID;
    	
    	/**
    	 * Constructor that initializes the userID.
    	 * @param userID The ID of the person whos connections we are sorting.
    	 */
    	public CustomComparatorName(String userID) {
    		this.userID = userID;
    	}
    	
    	/**
    	 * The method retrieves the id that does not equal the userID stored in the field.
    	 * Once that is obtained, we compare the last names of the other userIDs. If they are the same,
    	 * then we compare the first names. If the first names are the same, then we compare the ids themselves. If the
    	 * IDs are the same, then we return the result of comparing the dates.
    	 * 
    	 * @param c1 Connection one to compare.
    	 * @param c2 Connection two to compare.
    	 * 
    	 * @return A negative value if c1 should be before c2. A positive if c1 should be after c2.
    	 */
    	@Override
    	public int compare(Connection c1, Connection c2) {
    		String[] people = c1.getPeople();
    		String[] people2 = c2.getPeople();
    		
    		String otherID = (userID.equals(people[0])) ? people[1] : people[0];
    		String otherID2 = (userID.equals(people2[0])) ? people2[1] : people2[0];
    		
    		String lastName1 = mapOfPersons.get(otherID).getLast();
    		String lastName2 = mapOfPersons.get(otherID2).getLast();
    		
    		int lastNameResult = lastName1.compareTo(lastName2);
    		if(lastNameResult != 0) {
    			return lastNameResult;
    		}
    		
    		String firstName1 = mapOfPersons.get(otherID).getFirst();
    		String firstName2 = mapOfPersons.get(otherID2).getFirst();
    		
    		int firstNameResult = firstName1.compareTo(firstName2);
    		if(firstNameResult != 0) {
    			return firstNameResult;
    		}
    		
    		int userIDResult = otherID.compareTo(otherID2);
    		if(userIDResult != 0) {
    			return userIDResult;
    		}
    		
    		return c1.getDate().compareTo(c2.getDate());
    		
    	}
    }
    
    /**
     * Private helper class the helps sort two connections by their dates/IDs.
     */
    private class CustomComparatorDate implements Comparator<Connection> {
    	
    	/**
    	 * This method first compares the two dates of the connections. If the two connections
    	 * have the same date, then the result of comparing the two connection IDs is returned.
    	 * 
    	 * @param c1 Connection one to compare.
    	 * @param c2 Connection two to compare.
    	 * 
    	 * @return A negative integer if c1 should be before c2. A positive integer if c1 should be after c2.
    	 * Zero if the two connection IDs are the same.
    	 */
		@Override
		public int compare(Connection c1, Connection c2) {
			// Compare by date
			int dateComparison = c1.getDate().compareTo(c2.getDate());
			if (dateComparison != 0) {
				return dateComparison;
		    }
			// If dates are equal, compare by connection ID
			return c1.getId().compareTo(c2.getId());
		}
	}
    
    /**
     * Private helper class that helps sort a connection.
     */
    private class NameComparator implements Comparator<String> {
    	
    	/**
    	 * This method compares two users inside of a connection.
    	 * The method first compares the last names of the two users. If they are the same,
    	 * then their first names are compared. If they are the same, then the result of comparing
    	 * their IDs is returned.
    	 * 
    	 * @param userName1 The ID of the first user.
    	 * @param userName2 The ID of the second user.
    	 * 
    	 * @return A negative integer if userName1 comes before userName2. A positive integer if 
    	 * userName1 comes after userName2. Zero if they are the same and order is insignificant.
    	 */
    	@Override
    	public int compare(String userName1, String userName2) {
    		Person person1 = mapOfPersons.get(userName1);
    		Person person2 = mapOfPersons.get(userName2);
    		if(person1 == null && person2 == null) {
    			return 0;
    		} else if(person1 == null) {
    			return -1;
    		} else if(person2 == null) {
    			return 1;
    		}
    		
    		String lastName1 = person1.getLast();
    		String lastName2 = person2.getLast();
    		
    		int lastNameComparison = lastName1.compareTo(lastName2);
    		if(lastNameComparison != 0) {
    			return lastNameComparison;
    		}
    		
    		String firstName1 = person1.getFirst();
    		String firstName2 = person2.getFirst();
    		
    		int firstNameComparison = firstName1.compareTo(firstName2);
    		if(firstNameComparison != 0) {
    			return firstNameComparison;
    		}
    		
    		return userName1.compareTo(userName2);
    	}
    }
    
    
}
