<?php
$xml = simplexml_load_file("bookstore.xml");

//echo $_GET['title']

//$res = $xml->xpath("/bookstore/book[title = \"Everyday Italian\"]"); // works
//$res = $xml->xpath("/bookstore/book[title = "Everyday Italian"]");   // 500

$res = $xml->xpath("/bookstore/book[title = $_GET[title]]"); 
//$res = $xml->xpath("/bookstore/book[title = $_GET[title]]/author");

//$res = $xml->xpath("/bookstore/book[title = '" . $_GET['title'] . "']"); 

//$title = $_GET['title'];
//$res = $xml->xpath("/bookstore/book[title = '$title']"); 

//echo $res->author

?>