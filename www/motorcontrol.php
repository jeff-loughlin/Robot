<?php
$x = intval($_POST['x']);
$y = intval($_POST['y']);


//$x = 50 + (50 - $x);

$leftMotor = $y / 2;
$rightMotor = $y / 2;

$leftMotor -= ($x);// / 2);
$rightMotor += ($x);// / 2);

if ($leftMotor < -255 )
    $leftMotor = -255;
if ($leftMotor > 255)
    $leftMotor = 255;
if ($rightMotor < -255 )
    $rightMotor = -255;
if ($rightMotor > 255)
    $rightMotor = 255;

$out = "M:{$leftMotor},{$rightMotor}\r";
$myfile = fopen("RobotFifo", "w");
fwrite($myfile,$out);
fclose($myfile);
echo $out
?>
