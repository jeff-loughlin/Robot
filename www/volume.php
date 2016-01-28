<?php
$vol = $_POST['vol'];
$cmd = "amixer -c 0 sset PCM,0 $vol%,$vol%";

exec($cmd, $outputArray);
?>
