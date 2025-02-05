package edu.ncsu.csc316.social.manager;

import static org.junit.Assert.*;

import java.io.FileNotFoundException;

import org.junit.Test;

/**
 * This class handles testing the methods of the Java Class, ReportManager.
 * 
 * @author Charlie Austin (cjausti2)
 */
public class ReportManagerTest {
	
	/** Instance of ReportManager used for testing. */
	private ReportManager manager;
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
	 * Testing the method getConnectionByPerson with a medium data set.
	 * @throws FileNotFoundException If the specified file cannot be found.
	 */
	@Test
	public void testGetConnectionsByPerson() throws FileNotFoundException {
		// Regular reading of files. Large input.
		manager = new ReportManager(validPeopleTestFile, validConnectionsTestFile);
		String output = manager.getConnectionsByPerson();
		
		assertEquals("Connections for Phyliss Beahan (beahanp130) {\r\n"
				+ "   Justin Boyer (boyerj706) on LinkedIn since Sat Nov 20 19:11:47 EST 2021\r\n"
				+ "   Brooks Homenick (homenickb690) on Facebook since Sun Feb 11 05:45:35 EST 2018\r\n"
				+ "   Brooks Homenick (homenickb690) on LinkedIn since Sun Jun 23 13:13:15 EDT 2019\r\n"
				+ "   Leonard Runolfsdottir (runolfsdottirl591) on WhatsApp since Tue Dec 08 03:54:58 EST 2020\r\n"
				+ "   Royce Simonis (simonisr882) on Instagram since Sun May 10 21:10:38 EDT 2015\r\n"
				+ "}\r\n"
				+ "Connections for Justin Boyer (boyerj706) {\r\n"
				+ "   Phyliss Beahan (beahanp130) on LinkedIn since Sat Nov 20 19:11:47 EST 2021\r\n"
				+ "   Brooks Homenick (homenickb690) on Instagram since Wed Mar 11 06:10:33 EDT 2015\r\n"
				+ "   Cristobal Jones (jonesc957) on WeChat since Wed Nov 11 18:51:12 EST 2015\r\n"
				+ "   Napoleon Kertzmann (kertzmannn329) on Telegram since Wed Nov 16 03:42:00 EST 2016\r\n"
				+ "   Leonard Runolfsdottir (runolfsdottirl591) on WhatsApp since Mon Oct 26 22:11:06 EDT 2015\r\n"
				+ "   Leonard Runolfsdottir (runolfsdottirl591) on Discord since Sun Feb 04 09:23:24 EST 2018\r\n"
				+ "   Royce Simonis (simonisr882) on Twitch since Mon Aug 03 02:54:39 EDT 2020\r\n"
				+ "}\r\n"
				+ "Connections for Brooks Homenick (homenickb690) {\r\n"
				+ "   Phyliss Beahan (beahanp130) on Facebook since Sun Feb 11 05:45:35 EST 2018\r\n"
				+ "   Phyliss Beahan (beahanp130) on LinkedIn since Sun Jun 23 13:13:15 EDT 2019\r\n"
				+ "   Justin Boyer (boyerj706) on Instagram since Wed Mar 11 06:10:33 EDT 2015\r\n"
				+ "   Cristobal Jones (jonesc957) on Facebook since Wed Mar 11 00:26:46 EDT 2020\r\n"
				+ "   Napoleon Kertzmann (kertzmannn329) on Discord since Fri Jun 10 21:34:34 EDT 2016\r\n"
				+ "   Chang Murazik (murazikc970) on TikTok since Mon Apr 03 11:38:59 EDT 2023\r\n"
				+ "   Leonard Runolfsdottir (runolfsdottirl591) on Reddit since Sun Nov 10 23:02:36 EST 2019\r\n"
				+ "   Leonard Runolfsdottir (runolfsdottirl591) on Twitch since Sun Dec 19 07:42:48 EST 2021\r\n"
				+ "}\r\n"
				+ "Connections for Cristobal Jones (jonesc957) {\r\n"
				+ "   Justin Boyer (boyerj706) on WeChat since Wed Nov 11 18:51:12 EST 2015\r\n"
				+ "   Brooks Homenick (homenickb690) on Facebook since Wed Mar 11 00:26:46 EDT 2020\r\n"
				+ "   Chang Murazik (murazikc970) on LinkedIn since Fri Jul 02 22:41:29 EDT 2021\r\n"
				+ "}\r\n"
				+ "Connections for Napoleon Kertzmann (kertzmannn329) {\r\n"
				+ "   Justin Boyer (boyerj706) on Telegram since Wed Nov 16 03:42:00 EST 2016\r\n"
				+ "   Brooks Homenick (homenickb690) on Discord since Fri Jun 10 21:34:34 EDT 2016\r\n"
				+ "   Chang Murazik (murazikc970) on Telegram since Sun Nov 23 13:14:01 EST 2014\r\n"
				+ "   Royce Simonis (simonisr882) on Twitter since Wed Jul 28 22:55:33 EDT 2021\r\n"
				+ "}\r\n"
				+ "Connections for Chang Murazik (murazikc970) {\r\n"
				+ "   Brooks Homenick (homenickb690) on TikTok since Mon Apr 03 11:38:59 EDT 2023\r\n"
				+ "   Cristobal Jones (jonesc957) on LinkedIn since Fri Jul 02 22:41:29 EDT 2021\r\n"
				+ "   Napoleon Kertzmann (kertzmannn329) on Telegram since Sun Nov 23 13:14:01 EST 2014\r\n"
				+ "   Leonard Runolfsdottir (runolfsdottirl591) on Facebook since Fri Apr 18 20:45:17 EDT 2014\r\n"
				+ "}\r\n"
				+ "Connections for Leonard Runolfsdottir (runolfsdottirl591) {\r\n"
				+ "   Phyliss Beahan (beahanp130) on WhatsApp since Tue Dec 08 03:54:58 EST 2020\r\n"
				+ "   Justin Boyer (boyerj706) on WhatsApp since Mon Oct 26 22:11:06 EDT 2015\r\n"
				+ "   Justin Boyer (boyerj706) on Discord since Sun Feb 04 09:23:24 EST 2018\r\n"
				+ "   Brooks Homenick (homenickb690) on Reddit since Sun Nov 10 23:02:36 EST 2019\r\n"
				+ "   Brooks Homenick (homenickb690) on Twitch since Sun Dec 19 07:42:48 EST 2021\r\n"
				+ "   Chang Murazik (murazikc970) on Facebook since Fri Apr 18 20:45:17 EDT 2014\r\n"
				+ "}\r\n"
				+ "Connections for Royce Simonis (simonisr882) {\r\n"
				+ "   Phyliss Beahan (beahanp130) on Instagram since Sun May 10 21:10:38 EDT 2015\r\n"
				+ "   Justin Boyer (boyerj706) on Twitch since Mon Aug 03 02:54:39 EDT 2020\r\n"
				+ "   Napoleon Kertzmann (kertzmannn329) on Twitter since Wed Jul 28 22:55:33 EDT 2021\r\n"
				+ "}\r\n", output);
	}
	
	
	/**
	 * Tests the method getConnectionsByPerson with a small data set. Additionally, one person has no connections.
	 * @throws FileNotFoundException If the specified file cannot be found.
	 */
	@Test
	public void testGetConnectionsByPersonOneWithNoConections() throws FileNotFoundException {
		manager = new ReportManager(test1PeopleFile, test1ConnectionsFile);
		String output = manager.getConnectionsByPerson();
		
		assertEquals("Connections for Layla Crona (cronal634) {\r\n"
				+ "   Elroy Daniel (daniele34) on Instagram since Sun Dec 24 13:08:22 EST 2017\r\n"
				+ "   Rowena Price (pricer774) on LinkedIn since Sat Mar 12 11:49:52 EST 2016\r\n"
				+ "   Rowena Price (pricer774) on YouTube since Sun Feb 06 12:06:46 EST 2022\r\n"
				+ "}\r\n"
				+ "Connections for Elroy Daniel (daniele34) {\r\n"
				+ "   Layla Crona (cronal634) on Instagram since Sun Dec 24 13:08:22 EST 2017\r\n"
				+ "   John Daniel (danielj21) on Discord since Sun Dec 24 02:39:21 EST 2017\r\n"
				+ "   Santos McLaughlin (mclaughlins441) on LinkedIn since Tue Oct 25 01:14:24 EDT 2022\r\n"
				+ "   Santos McLaughlin (mclaughlins441) on Discord since Mon Jan 02 15:33:47 EST 2023\r\n"
				+ "   Rowena Price (pricer774) on WeChat since Fri Oct 04 23:39:25 EDT 2013\r\n"
				+ "}\r\n"
				+ "Connections for John Daniel (danielj01) {\r\n"
				+ "   John Daniel (danielj21) on Discord since Wed Mar 01 13:25:47 EST 2023\r\n"
				+ "}\r\n"
				+ "Connections for John Daniel (danielj21) {\r\n"
				+ "   Elroy Daniel (daniele34) on Discord since Sun Dec 24 02:39:21 EST 2017\r\n"
				+ "   John Daniel (danielj01) on Discord since Wed Mar 01 13:25:47 EST 2023\r\n"
				+ "}\r\n"
				+ "Connections for Santos McLaughlin (mclaughlins441) {\r\n"
				+ "   Elroy Daniel (daniele34) on LinkedIn since Tue Oct 25 01:14:24 EDT 2022\r\n"
				+ "   Elroy Daniel (daniele34) on Discord since Mon Jan 02 15:33:47 EST 2023\r\n"
				+ "   Rowena Price (pricer774) on LinkedIn since Mon Jul 06 10:26:48 EDT 2020\r\n"
				+ "   Rowena Price (pricer774) on Snapchat since Sat Jul 18 23:30:45 EDT 2020\r\n"
				+ "}\r\n"
				+ "Connections for Rowena Price (pricer774) {\r\n"
				+ "   Layla Crona (cronal634) on LinkedIn since Sat Mar 12 11:49:52 EST 2016\r\n"
				+ "   Layla Crona (cronal634) on YouTube since Sun Feb 06 12:06:46 EST 2022\r\n"
				+ "   Elroy Daniel (daniele34) on WeChat since Fri Oct 04 23:39:25 EDT 2013\r\n"
				+ "   Santos McLaughlin (mclaughlins441) on LinkedIn since Mon Jul 06 10:26:48 EDT 2020\r\n"
				+ "   Santos McLaughlin (mclaughlins441) on Snapchat since Sat Jul 18 23:30:45 EDT 2020\r\n"
				+ "}\r\n"
				+ "Connections for Tonita Schumm (schummt809) {\r\n"
				+ "   No connections exist\r\n"
				+ "}\r\n", output);
	}
	
	/**
	 * Tests ReportManager with an input file that contains no people.
	 * @throws FileNotFoundException If the specified file cannot be found.
	 */
	@Test
	public void testNoPeopleFound() throws FileNotFoundException {
		manager = new ReportManager(test2PeopleFile, test1ConnectionsFile);
		assertEquals("No people information was provided.\r\n", manager.getConnectionsByPerson());
	}
	
	/**
	 * Tests ReportManager with an input file that contains no connections.
	 * @throws FileNotFoundException If the specified file cannot be found.
	 */
	@Test
	public void testNoConnectionsFound() throws FileNotFoundException {
		manager = new ReportManager(test1PeopleFile, test2ConnectionsFile);
		assertEquals("No connections exist in the social media network.\r\n", manager.getConnectionsByPerson());
	}
	
	/**
	 * Tests the method getConnectionsByPlatform with a large data set.
	 * @throws FileNotFoundException If the specified file cannot be found.
	 */
	@Test
	public void testGetConnectionsByPlatform() throws FileNotFoundException {
		manager = new ReportManager(validPeopleTestFile, validConnectionsTestFile);
		String output = manager.getConnectionsByPlatform();
		
		assertEquals("Connections on Discord {\r\n"
				+ "   Fri Jun 10 21:34:34 EDT 2016: Brooks Homenick (homenickb690) <--> Napoleon Kertzmann (kertzmannn329)\r\n"
				+ "   Sun Feb 04 09:23:24 EST 2018: Justin Boyer (boyerj706) <--> Leonard Runolfsdottir (runolfsdottirl591)\r\n"
				+ "}\r\n"
				+ "Connections on Facebook {\r\n"
				+ "   Fri Apr 18 20:45:17 EDT 2014: Chang Murazik (murazikc970) <--> Leonard Runolfsdottir (runolfsdottirl591)\r\n"
				+ "   Sun Feb 11 05:45:35 EST 2018: Phyliss Beahan (beahanp130) <--> Brooks Homenick (homenickb690)\r\n"
				+ "   Wed Mar 11 00:26:46 EDT 2020: Brooks Homenick (homenickb690) <--> Cristobal Jones (jonesc957)\r\n"
				+ "}\r\n"
				+ "Connections on Instagram {\r\n"
				+ "   Wed Mar 11 06:10:33 EDT 2015: Justin Boyer (boyerj706) <--> Brooks Homenick (homenickb690)\r\n"
				+ "   Sun May 10 21:10:38 EDT 2015: Phyliss Beahan (beahanp130) <--> Royce Simonis (simonisr882)\r\n"
				+ "}\r\n"
				+ "Connections on LinkedIn {\r\n"
				+ "   Sun Jun 23 13:13:15 EDT 2019: Phyliss Beahan (beahanp130) <--> Brooks Homenick (homenickb690)\r\n"
				+ "   Fri Jul 02 22:41:29 EDT 2021: Cristobal Jones (jonesc957) <--> Chang Murazik (murazikc970)\r\n"
				+ "   Sat Nov 20 19:11:47 EST 2021: Phyliss Beahan (beahanp130) <--> Justin Boyer (boyerj706)\r\n"
				+ "}\r\n"
				+ "Connections on Reddit {\r\n"
				+ "   Sun Nov 10 23:02:36 EST 2019: Brooks Homenick (homenickb690) <--> Leonard Runolfsdottir (runolfsdottirl591)\r\n"
				+ "}\r\n"
				+ "Connections on Telegram {\r\n"
				+ "   Sun Nov 23 13:14:01 EST 2014: Napoleon Kertzmann (kertzmannn329) <--> Chang Murazik (murazikc970)\r\n"
				+ "   Wed Nov 16 03:42:00 EST 2016: Justin Boyer (boyerj706) <--> Napoleon Kertzmann (kertzmannn329)\r\n"
				+ "}\r\n"
				+ "Connections on TikTok {\r\n"
				+ "   Mon Apr 03 11:38:59 EDT 2023: Brooks Homenick (homenickb690) <--> Chang Murazik (murazikc970)\r\n"
				+ "}\r\n"
				+ "Connections on Twitch {\r\n"
				+ "   Mon Aug 03 02:54:39 EDT 2020: Justin Boyer (boyerj706) <--> Royce Simonis (simonisr882)\r\n"
				+ "   Sun Dec 19 07:42:48 EST 2021: Brooks Homenick (homenickb690) <--> Leonard Runolfsdottir (runolfsdottirl591)\r\n"
				+ "}\r\n"
				+ "Connections on Twitter {\r\n"
				+ "   Wed Jul 28 22:55:33 EDT 2021: Napoleon Kertzmann (kertzmannn329) <--> Royce Simonis (simonisr882)\r\n"
				+ "}\r\n"
				+ "Connections on WeChat {\r\n"
				+ "   Wed Nov 11 18:51:12 EST 2015: Justin Boyer (boyerj706) <--> Cristobal Jones (jonesc957)\r\n"
				+ "}\r\n"
				+ "Connections on WhatsApp {\r\n"
				+ "   Mon Oct 26 22:11:06 EDT 2015: Justin Boyer (boyerj706) <--> Leonard Runolfsdottir (runolfsdottirl591)\r\n"
				+ "   Tue Dec 08 03:54:58 EST 2020: Phyliss Beahan (beahanp130) <--> Leonard Runolfsdottir (runolfsdottirl591)\r\n"
				+ "}\r\n", output);
	}
	
	/**
	 * Tests the method getConnectionsByPlatform with a smaller data set.
	 * @throws FileNotFoundException If the specified file cannot be found.
	 */
	@Test
	public void testGetConnectionsByPlatformSmallerDataSet() throws FileNotFoundException {
		manager = new ReportManager(test1PeopleFile, test1ConnectionsFile);
		String output = manager.getConnectionsByPlatform();
		
		assertEquals("Connections on Discord {\r\n"
				+ "   Sun Dec 24 02:39:21 EST 2017: Elroy Daniel (daniele34) <--> John Daniel (danielj21)\r\n"
				+ "   Mon Jan 02 15:33:47 EST 2023: Elroy Daniel (daniele34) <--> Santos McLaughlin (mclaughlins441)\r\n"
				+ "   Wed Mar 01 13:25:47 EST 2023: John Daniel (danielj01) <--> John Daniel (danielj21)\r\n"
				+ "}\r\n"
				+ "Connections on Instagram {\r\n"
				+ "   Sun Dec 24 13:08:22 EST 2017: Layla Crona (cronal634) <--> Elroy Daniel (daniele34)\r\n"
				+ "}\r\n"
				+ "Connections on LinkedIn {\r\n"
				+ "   Sat Mar 12 11:49:52 EST 2016: Layla Crona (cronal634) <--> Rowena Price (pricer774)\r\n"
				+ "   Mon Jul 06 10:26:48 EDT 2020: Santos McLaughlin (mclaughlins441) <--> Rowena Price (pricer774)\r\n"
				+ "   Tue Oct 25 01:14:24 EDT 2022: Elroy Daniel (daniele34) <--> Santos McLaughlin (mclaughlins441)\r\n"
				+ "}\r\n"
				+ "Connections on Snapchat {\r\n"
				+ "   Sat Jul 18 23:30:45 EDT 2020: Santos McLaughlin (mclaughlins441) <--> Rowena Price (pricer774)\r\n"
				+ "}\r\n"
				+ "Connections on WeChat {\r\n"
				+ "   Fri Oct 04 23:39:25 EDT 2013: Elroy Daniel (daniele34) <--> Rowena Price (pricer774)\r\n"
				+ "}\r\n"
				+ "Connections on YouTube {\r\n"
				+ "   Sun Feb 06 12:06:46 EST 2022: Layla Crona (cronal634) <--> Rowena Price (pricer774)\r\n"
				+ "}\r\n", output);
	}
	

	/**
	 * Tests a super small data-set where date of connection are the same so IDs must be compared.
	 * 
	 * @throws FileNotFoundException If the specified file cannot be found.
	 */
	@Test
	public void testCompareByIDs() throws FileNotFoundException {
		manager = new ReportManager(test3PeopleFile, test3ConnectionsFile);
		String output = manager.getConnectionsByPlatform();
		
		assertEquals("Connections on Snapchat {\r\n"
				+ "   Sat Jul 18 23:30:45 EDT 2020: Elroy Daniel (daniele34) <--> Santos McLaughlin (mclaughlins441)\r\n"
				+ "   Sat Jul 18 23:30:45 EDT 2020: Santos McLaughlin (mclaughlins441) <--> Rowena Price (pricer774)\r\n"
				+ "}\r\n",
				output);
	}
}
