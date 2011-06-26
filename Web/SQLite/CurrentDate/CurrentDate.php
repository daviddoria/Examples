<html>
<body>

<?php
try
{
  $database = new PDO('sqlite:Test.db');
  //$myquery = 'select * from TestTable'; // all dates
  $myquery = 'select * from TestTable WHERE date(\'now\') < DateColumn'; // only dates after right now
  $result = $database->query($myquery);

  foreach($result as $row)
  {
    echo $row['DateColumn'] . '<br/>';
  }

}
catch(Exception $e)
{
  die('Could not connect to database.');
}
?>

</body>
</html>
