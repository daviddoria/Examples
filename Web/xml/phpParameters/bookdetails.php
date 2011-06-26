<?php
$xml = simplexml_load_file("bookstore.xml");

echo "<br/>Received param: " . $_GET["title"];

echo "<br/>Book hardcoded:";
$res = $xml->xpath("/bookstore/book[title = 'TestBook']");
echo $res[0]->author;


echo "<br/>Book of param:";
$res = $xml->xpath("/bookstore/book[title = '".$_GET["title"]."']");
echo $res[0]->author;

?>