<?php
$x = intval($_POST['x']);
$y = intval($_POST['y']);

$out = "PT:{$x},{$y}\r";
$myfile = fopen("RobotFifo", "w");
fwrite($myfile,$out);
fclose($myfile);
echo $out
?>
