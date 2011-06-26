<?php
$xml = simplexml_load_file("bookstore.xml");

foreach($xml->children() as $child)
  {
  //echo $child->title . "<br/>";
  //echo "<a href=" . urlencode("bookdetails.php?title=" . $child->title) . ">" . $child->title . "</a> <br/>";
  echo "<a href=bookdetails.php?title=" . urlencode($child->title) . ">" . $child->title . "</a> <br/>";
  }
?>