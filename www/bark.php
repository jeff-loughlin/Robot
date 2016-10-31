<?php
$cmd = "/usr/bin/aplay dog-sounds/bark-0" . rand(1,7) . ".wav";
exec($cmd, $outputArray);
?>
