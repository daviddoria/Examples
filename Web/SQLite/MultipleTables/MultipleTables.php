<?php
try
{
  // open the database
  $database = new PDO('sqlite:database.db');

// get a list of all of the tables
$myquery = 'SELECT name FROM sqlite_master
WHERE type IN (\'table\',\'view\') AND name NOT LIKE \'sqlite_%\'
UNION ALL
SELECT name FROM sqlite_temp_master
WHERE type IN (\'table\',\'view\')
ORDER BY 1';

  $result = $database->query($myquery);

// display the list
  foreach($result as $row)
  {
  echo $row[0] . '<br/>';
  }

}
catch(Exception $e)
{
  die('Could not connect to database.');
}
?>