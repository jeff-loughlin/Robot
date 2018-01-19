<?php
$cmd = "/usr/bin/omxplayer cat-sounds/meow-00" . rand(1,8) . ".wav";
exec($cmd, $outputArray);
?>
