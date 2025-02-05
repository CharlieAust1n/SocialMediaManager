# SocialMediaManager
The SocialMediaManager was the final project for my CSC 316 - Data Structures & Algoriths class at North Carolina State University.

## What The Manager Does
The SocialMediaManager parses two files: a file of connections and a file of people. The manager retrieves the data in the files and
displays connections depending on what the user requests. 
* If the user requested to view connections of each social media user, then the manager
displays the connections linked to a social media user in an orderly fashion. The order is first determined by name, then by user Id, and then by date of connection.
* If the user requested to view connections for each social media platform, then the manager displays the connections for all social meida platforms. Each list of connection is sorted
by date. If the dates are the same, then they are sorted by the connection ID. For the individual connection, the order of people is sorted by their names. If the names are
the same, then it is sorted by ID.

## Constraints
* Cannot use the hash map data structure.
* Must use custom made data structures. Can be found in this repo: https://github.com/CharlieAust1n/CSC-316-DataStructures/tree/main
* Must be able to finish running in under 30 seconds when given a very large file.
