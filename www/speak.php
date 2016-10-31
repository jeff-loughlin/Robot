<?php
$text = $_POST['text'];
$speed = $_POST['speed'];
$cmd = "/usr/bin/espeak --stdout -s $speed \"$text\" | /usr/bin/aplay";
exec($cmd, $outputArray);
?>
