<?php
$cmd = $_POST['cmd'];

$myfile = fopen("RobotFifo", "w");
$out = "{$cmd}\r";
fwrite($myfile,$out);
fclose($myfile);
echo $out
?>
