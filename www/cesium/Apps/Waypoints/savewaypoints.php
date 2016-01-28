<?php
$waypoints = $_POST['outArray'];

$waypointFile = fopen("waypoints.dat","wb");
foreach ($waypoints as $waypoint)
{
    fwrite($waypointFile, $waypoint);
    fwrite($waypointFile, "\n");
}
fclose($waypointFile);

header('Location: ../../../ctrl.php');

?>
