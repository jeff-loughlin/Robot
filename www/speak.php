<?php
$text = $_POST['text'];
$speed = $_POST['speed'];
$cmd = "/usr/bin/espeak -s $speed \"$text\"";
exec($cmd, $outputArray);
?>
