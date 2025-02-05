package edu.ncsu.csc316.social.ui;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

import edu.ncsu.csc316.social.manager.ReportManager;

/**
 * This is the highest level class in the program. SocialMediaManagerUI handles providing
 * a command-line interface for interaction with a social media management system. Users can view
 * connections of users on different social media platforms. Users can also view all connections that each platform has.
 * 
 * @author Charlie Austin (cjausti2)
 */
public class SocialMediaManagerUI {
	
	/** The ReportManager instance used for managing social media connections. */
	private static ReportManager manager;
	/** The folder where input files are stored. */
	private static final String INPUT_FOLDER = "input";
	
	/**
	 * This is where the program starts. The user is initially prompted to enter in 
	 * a valid file path for both people and connections. If the file path is invalid, the user
	 * is re-prompted to enter a valid file path. After both files have been read in, the terminal prompts the user
	 * to choose a command: view connections of each social media user (VCSMU), view connections for each social media platform (VCSMP), or
	 * quit. The program will continue to ask for a command until quit is given or the program is forcibly terminated.
	 * @param args The command-line arguments.
	 */
	public static void main(String[] args) {
		
		Scanner in = new Scanner(System.in);
		
		File personFile = null;
		File connectionFile = null;
		
		while(personFile == null) {
			System.out.print("Please enter the name of the person file: ");
			String personFileName = in.next();
			personFile = new File(INPUT_FOLDER, personFileName);
			if(!personFile.exists()) {
				System.out.println("File not found. Please try again.");
				personFile = null;
			}
				
		}
		
		while(connectionFile == null) {
			System.out.print("Please enter the name of the connection file: ");
			String connectionFileName = in.next();
			connectionFile = new File(INPUT_FOLDER, connectionFileName);
			if(!connectionFile.exists()) {
				System.out.println("File not found. Please try again.");
				connectionFile = null;
			}
				
		}
		
		try {
			manager = new ReportManager(personFile.getAbsolutePath(), connectionFile.getAbsolutePath());
		} catch(FileNotFoundException e) {
			System.out.println("File not found\n");
		}
		
		String command = "";
		
		while(!command.equals("quit")) {
			System.out.println("\nSocial Media Manager Program - Please choose a command. \n");
			System.out.println("VCSMU - View Connections of Each Social Media User\n"
					+ "VCSMP - View Connections for Each Social Media Platform\n"
					+ "quit\n");
			System.out.print("cmd> ");
			command = in.next().toLowerCase();
			
			switch(command) {
				case "vcsmu":
					System.out.println("");
					System.out.println(manager.getConnectionsByPerson());
					break;
				case "vcsmp":
					System.out.println("");
					System.out.println(manager.getConnectionsByPlatform());
					break;
				case "quit":
					break;
				default:
					System.out.println("Invalid command! Please re-enter command.");
					break;
			}
		}
		in.close();
		System.exit(0);
	}
}
