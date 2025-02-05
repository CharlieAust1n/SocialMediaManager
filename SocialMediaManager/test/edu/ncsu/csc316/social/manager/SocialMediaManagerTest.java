package edu.ncsu.csc316.social.manager;

import static org.junit.Assert.*;

import java.io.FileNotFoundException;
import java.util.Iterator;

import org.junit.Test;

import edu.ncsu.csc316.dsa.list.List;
import edu.ncsu.csc316.dsa.map.Map;
import edu.ncsu.csc316.dsa.map.Map.Entry;
import edu.ncsu.csc316.social.data.Connection;
import edu.ncsu.csc316.social.data.Person;
import edu.ncsu.csc316.social.dsa.DSAFactory;

/**
 * This class handles testing the Java Class, SocialMediaManager.
 * 
 * @author Charlie Austin (cjausti2)
 */
public class SocialMediaManagerTest {
	/** Instance of SocialMediaManager used for testing. */
	private SocialMediaManager manager;
	/** A map of person IDS to Person objects, representing individuals in the social media network. */
	private Map<String, Person> mapOfPersons;
	/** Input file containing a medium sized data set of people. */
	private String validPeopleTestFile = "input/people.csv";
	/** Input file containing a medium sized data set of connections */
	private String validConnectionsTestFile = "input/connections.csv";
	/** Input file containing a small sized data set of people. */
	private String test1PeopleFile = "input/test-1-people.csv";
	/** Input file containing a small sized data set of connections. */
	private String test1ConnectionsFile = "input/test-1-connections.csv";
	/** Input file of people that contains no people. */
	private String test2PeopleFile = "input/test-2-people.csv";
	/** Input file of connections that contains no connections */
	private String test2ConnectionsFile = "input/test-2-connections.csv";
	/** Input file of people. Super small data set. */
	private String test3PeopleFile = "input/test-3-people.csv";
	/** Input file of connections where dates are the same. Super small data set. */
	private String test3ConnectionsFile = "input/test-3-connections.csv";
	
	/**
	 * Tests the method getPeople.
	 * @throws FileNotFoundException If the specified file cannot be found.
	 */
	@Test
	public void testGetPeople() throws FileNotFoundException {
		manager = new SocialMediaManager(validPeopleTestFile, validConnectionsTestFile);
		mapOfPersons = manager.getPeople();
		assertFalse(mapOfPersons.isEmpty());
		
		Iterator<Entry<String, Person>> it = mapOfPersons.entrySet().iterator();
		
		assertTrue(it.hasNext());
		Entry<String, Person> entry = it.next();
		
		assertEquals("beahanp130", (String)entry.getKey());
		assertEquals("Phyliss", (String)entry.getValue().getFirst());
		assertEquals("Beahan", (String)entry.getValue().getLast());
		
		assertTrue(it.hasNext());
		entry = it.next();
		assertEquals("boyerj706", (String)entry.getKey());
		assertEquals("Justin", (String)entry.getValue().getFirst());
		assertEquals("Boyer", (String)entry.getValue().getLast());
	}

	/**
	 * Tests the method getConnectionsByPerson with a medium sized data set.
	 * @throws FileNotFoundException If the specified file cannot be found.
	 */
	@Test
	public void testGetConnectionsByPerson() throws FileNotFoundException {
		manager = new SocialMediaManager(validPeopleTestFile, validConnectionsTestFile);
		Map<String, List<Connection>> connectionsByPersonMap = DSAFactory.getMap(null);
		connectionsByPersonMap = manager.getConnectionsByPerson();
		
		Iterator<Entry<String, List<Connection>>> it = connectionsByPersonMap.entrySet().iterator();
		assertTrue(it.hasNext());
		Entry<String, List<Connection>> entry = it.next();
		assertEquals("beahanp130", (String)entry.getKey());
		assertEquals(5, entry.getValue().size());
		assertEquals("Connection [id=MMNI3885, people=[boyerj706, beahanp130], platform=LinkedIn, date=Sat Nov 20 19:11:47 EST 2021]", 
				(String)entry.getValue().get(0).toString());
		assertEquals("Connection [id=WPEM8662, people=[beahanp130, homenickb690], platform=Facebook, date=Sun Feb 11 05:45:35 EST 2018]", 
				(String)entry.getValue().get(1).toString());
		assertEquals("Connection [id=DVIQ9879, people=[homenickb690, beahanp130], platform=LinkedIn, date=Sun Jun 23 13:13:15 EDT 2019]",
				(String)entry.getValue().get(2).toString());
		assertEquals("Connection [id=FBGH9119, people=[runolfsdottirl591, beahanp130], platform=WhatsApp, date=Tue Dec 08 03:54:58 EST 2020]",
				(String)entry.getValue().get(3).toString());
		assertEquals("Connection [id=YNTP0089, people=[beahanp130, simonisr882], platform=Instagram, date=Sun May 10 21:10:38 EDT 2015]",
				(String)entry.getValue().get(4).toString());
		
		assertTrue(it.hasNext());
		entry = it.next();
		assertEquals("boyerj706", (String)entry.getKey());
		assertEquals(7, entry.getValue().size());
		assertEquals("Connection [id=MMNI3885, people=[boyerj706, beahanp130], platform=LinkedIn, date=Sat Nov 20 19:11:47 EST 2021]", 
				(String)entry.getValue().get(0).toString());
		
		assertTrue(it.hasNext());
		entry = it.next();
		assertEquals("homenickb690", (String)entry.getKey());
		assertEquals(8, entry.getValue().size());
		assertEquals("Connection [id=WPEM8662, people=[beahanp130, homenickb690], platform=Facebook, date=Sun Feb 11 05:45:35 EST 2018]",
				(String)entry.getValue().get(0).toString());
		
		assertTrue(it.hasNext());
		entry = it.next();
		assertEquals("jonesc957", (String)entry.getKey());
		assertEquals(3, entry.getValue().size());
		assertEquals("Connection [id=FLPB3202, people=[jonesc957, boyerj706], platform=WeChat, date=Wed Nov 11 18:51:12 EST 2015]",
				(String)entry.getValue().get(0).toString());
		
		
		assertTrue(it.hasNext());
		entry = it.next();
		assertEquals("kertzmannn329", (String)entry.getKey());
		assertEquals(4, entry.getValue().size());
		assertEquals("Connection [id=WTSO7763, people=[boyerj706, kertzmannn329], platform=Telegram, date=Wed Nov 16 03:42:00 EST 2016]",
				(String)entry.getValue().get(0).toString());
		
		assertTrue(it.hasNext());
		entry = it.next();
		assertEquals("murazikc970", (String)entry.getKey());
		assertEquals(4, entry.getValue().size());
		assertEquals("Connection [id=JOJK3526, people=[homenickb690, murazikc970], platform=TikTok, date=Mon Apr 03 11:38:59 EDT 2023]",
				(String)entry.getValue().get(0).toString());
		
		assertTrue(it.hasNext());
		entry = it.next();
		assertEquals("runolfsdottirl591", (String)entry.getKey());
		assertEquals(6, entry.getValue().size());
		assertEquals("Connection [id=FBGH9119, people=[runolfsdottirl591, beahanp130], platform=WhatsApp, date=Tue Dec 08 03:54:58 EST 2020]",
				(String)entry.getValue().get(0).toString());
		
		assertTrue(it.hasNext());
		entry = it.next();
		assertEquals("simonisr882", (String)entry.getKey());
		assertEquals(3, entry.getValue().size());
		assertEquals("Connection [id=YNTP0089, people=[beahanp130, simonisr882], platform=Instagram, date=Sun May 10 21:10:38 EDT 2015]",
				(String)entry.getValue().get(0).toString());
		
		assertFalse(it.hasNext());
		
	}

	/**
	 * Tests the method getConnectionsByPlatform with a medium sized data set.
	 * @throws FileNotFoundException If the specified file cannot be found.
	 */
	@Test
	public void testGetConnectionsByPlatform() throws FileNotFoundException {
		manager = new SocialMediaManager(validPeopleTestFile, validConnectionsTestFile);
		Map<String, List<Connection>> connectionsByPlatformMap = DSAFactory.getMap(null);
		connectionsByPlatformMap = manager.getConnectionsByPlatform();
		
		Iterator<Entry<String, List<Connection>>> it = connectionsByPlatformMap.entrySet().iterator();
		assertTrue(it.hasNext());
		Entry<String, List<Connection>> entry = it.next();
		assertEquals("Discord", (String)entry.getKey());
		assertEquals("Connection [id=XAFL8903, people=[homenickb690, kertzmannn329], platform=Discord, date=Fri Jun 10 21:34:34 EDT 2016]", 
				(String)entry.getValue().get(0).toString());
		assertEquals("Connection [id=SAFX7881, people=[boyerj706, runolfsdottirl591], platform=Discord, date=Sun Feb 04 09:23:24 EST 2018]",
				(String)entry.getValue().get(1).toString());
		
		assertTrue(it.hasNext());
		entry = it.next();
		assertEquals("Facebook", (String)entry.getKey());
		assertEquals("Connection [id=SCEN2300, people=[murazikc970, runolfsdottirl591], platform=Facebook, date=Fri Apr 18 20:45:17 EDT 2014]",
				(String)entry.getValue().get(0).toString());
		assertEquals(3, entry.getValue().size());
		
		assertTrue(it.hasNext());
		entry = it.next();
		assertEquals("Instagram", (String)entry.getKey());
		assertEquals(2, entry.getValue().size());
		assertEquals("Connection [id=LZJN5229, people=[boyerj706, homenickb690], platform=Instagram, date=Wed Mar 11 06:10:33 EDT 2015]",
				(String)entry.getValue().get(0).toString());
		
		assertTrue(it.hasNext());
		entry = it.next();
		assertEquals("LinkedIn", (String)entry.getKey());
		assertEquals(3, entry.getValue().size());
		assertEquals("Connection [id=DVIQ9879, people=[beahanp130, homenickb690], platform=LinkedIn, date=Sun Jun 23 13:13:15 EDT 2019]",
				(String)entry.getValue().get(0).toString());
		
		assertTrue(it.hasNext());
		entry = it.next();
		assertEquals("Reddit", (String)entry.getKey());
		assertEquals(1, entry.getValue().size());
		assertEquals("Connection [id=TUYK3389, people=[homenickb690, runolfsdottirl591], platform=Reddit, date=Sun Nov 10 23:02:36 EST 2019]",
				(String)entry.getValue().get(0).toString());
		
		assertTrue(it.hasNext());
		entry = it.next();
		assertEquals("Telegram", (String)entry.getKey());
		assertEquals(2, entry.getValue().size());
		assertEquals("Connection [id=RUIN7780, people=[kertzmannn329, murazikc970], platform=Telegram, date=Sun Nov 23 13:14:01 EST 2014]",
				(String)entry.getValue().get(0).toString());
		
		assertTrue(it.hasNext());
		entry = it.next();
		assertEquals("TikTok", (String)entry.getKey());
		assertEquals(1, entry.getValue().size());
		assertEquals("Connection [id=JOJK3526, people=[homenickb690, murazikc970], platform=TikTok, date=Mon Apr 03 11:38:59 EDT 2023]",
				(String)entry.getValue().get(0).toString());
		
		assertTrue(it.hasNext());
		entry = it.next();
		assertEquals("Twitch", (String)entry.getKey());
		assertEquals(2, entry.getValue().size());
		assertEquals("Connection [id=HQLO6804, people=[boyerj706, simonisr882], platform=Twitch, date=Mon Aug 03 02:54:39 EDT 2020]",
				(String)entry.getValue().get(0).toString());
		
		assertTrue(it.hasNext());
		entry = it.next();
		assertEquals("Twitter", (String)entry.getKey());
		assertEquals(1, entry.getValue().size());
		assertEquals("Connection [id=QOYT6634, people=[kertzmannn329, simonisr882], platform=Twitter, date=Wed Jul 28 22:55:33 EDT 2021]",
				(String)entry.getValue().get(0).toString());
		
		assertTrue(it.hasNext());
		entry = it.next();
		assertEquals("WeChat", (String)entry.getKey());
		assertEquals(1, entry.getValue().size());
		assertEquals("Connection [id=FLPB3202, people=[boyerj706, jonesc957], platform=WeChat, date=Wed Nov 11 18:51:12 EST 2015]",
				(String)entry.getValue().get(0).toString());
		
		assertTrue(it.hasNext());
		entry = it.next();
		assertEquals("WhatsApp", (String)entry.getKey());
		assertEquals(2, entry.getValue().size());
		assertEquals("Connection [id=UYSN4906, people=[boyerj706, runolfsdottirl591], platform=WhatsApp, date=Mon Oct 26 22:11:06 EDT 2015]",
				(String)entry.getValue().get(0).toString());
		
		assertFalse(it.hasNext());
	}

	/**
	 * Tests the ReportManager function when a person file contains no persons.
	 * @throws FileNotFoundException If the specified file cannot be found.
	 */
	@Test
	public void testNoPeopleFound() throws FileNotFoundException {
		manager = new SocialMediaManager(test2PeopleFile, test1ConnectionsFile);
		assertTrue(manager.getConnectionsByPerson().isEmpty());
	}
	
	/**
	 * Tests the ReportManager function when a connection file contains no connections.
	 * @throws FileNotFoundException If the specified file cannot be found.
	 */
	@Test
	public void testNoConnectionsFound() throws FileNotFoundException {
		manager = new SocialMediaManager(test1PeopleFile, test2ConnectionsFile);
		Map<String, List<Connection>> connectionsByPersonMap = DSAFactory.getMap(null);
		connectionsByPersonMap = manager.getConnectionsByPerson();
		Iterator<Entry<String, List<Connection>>> it = connectionsByPersonMap.entrySet().iterator();
		assertFalse(it.hasNext());
	}
	
	/**
	 * Tests a super small data-set where date of connection are the same so IDs must be compared.
	 * 
	 * @throws FileNotFoundException If the specified file cannot be found.
	 */
	@Test
	public void testCompareByIDs() throws FileNotFoundException {
		manager = new SocialMediaManager(test3PeopleFile, test3ConnectionsFile);
		Map<String, List<Connection>> connectionsByPlatformMap = DSAFactory.getMap(null);
		connectionsByPlatformMap = manager.getConnectionsByPlatform();
		
		Iterator<Entry<String, List<Connection>>> it = connectionsByPlatformMap.entrySet().iterator();
		assertTrue(it.hasNext());
		Entry<String, List<Connection>> entry = it.next();
		assertEquals("Snapchat", (String)entry.getKey());
		assertEquals(2, entry.getValue().size());
		assertEquals("SWRF2156", entry.getValue().get(0).getId());
		assertEquals("Connection [id=SWRF2156, people=[daniele34, mclaughlins441], platform=Snapchat, date=Sat Jul 18 23:30:45 EDT 2020]",
				(String)entry.getValue().get(0).toString());
		assertEquals("YGWT2496", entry.getValue().get(1).getId());
		assertEquals("Connection [id=YGWT2496, people=[mclaughlins441, pricer774], platform=Snapchat, date=Sat Jul 18 23:30:45 EDT 2020]",
				(String)entry.getValue().get(1).toString());
	}
}
