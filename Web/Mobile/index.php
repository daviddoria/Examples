<!DOCTYPE HTML PUBLIC "-//W3C//DTD HTML 4.0 Transitional//EN">
<html>
<head>
<title>Your Page Title</title>

<?php
require_once('mobile_device_detect.php');
$mobile = mobile_device_detect();
if($mobile)
{
echo '<meta http-equiv="REFRESH" content="0;url=http://www.yahoo.com"></HEAD>';
}
else
{
echo '<meta http-equiv="REFRESH" content="0;url=http://www.google.com"></HEAD>';
}
?>
</head>
<body>
</body>
</html>