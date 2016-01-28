var viewer = new Cesium.Viewer('cesiumContainer');

/*
var prevWaypoint = {
	latitude:'40.085573',
	longitude: '-75.336789'
};

var startingPoint = {
	latitude: prevWaypoint.latitude,
	longitude: prevWaypoint.longitude
};
*/
var startingPoint = {
	latitude:'40.085573',
	longitude: '-75.336789'
};
var prevWaypoint = {
	latitude: startingPoint.latitude,
	longitude: startingPoint.longitude
};

getCurrentLocation();
var prevWaypoint = {
	latitude: startingPoint.latitude,
	longitude: startingPoint.longitude
};
var waypoints = [];
waypoints.push(startingPoint);

var numPoints = 1;

var myLocation = viewer.entities.add({
        position: Cesium.Cartesian3.fromDegrees(prevWaypoint.longitude, prevWaypoint.latitude, 150000.0),
        name : 'Current Location',
        id : 0,
        ellipse : {
            semiMinorAxis : 1.0,
            semiMajorAxis : 1.0,
            height: 0.0,
            material : Cesium.Color.RED.withAlpha(0.5)
        },
	waypoint : waypoints[0]
    });

viewer.flyTo(viewer.entities, {
        offset : new Cesium.HeadingPitchRange(0,-Math.PI / 2, 200),
        duration : 0.0
    });


viewer.canvas.addEventListener('click', mouseClickHandler, false);


function getCurrentLocation()
{
    var rawFile = new XMLHttpRequest();
    rawFile.open("GET", "telemetry.dat", false);
    rawFile.onreadystatechange = function ()
    {
        if(rawFile.readyState === 4)
        {
            if(rawFile.status === 200 || rawFile.status == 0)
            {
                var str = rawFile.responseText;
                telemetry = str.split("|");
		startingPoint.latitude = telemetry[0];
		startingPoint.longitude = telemetry[1];
            }
        }
    }
    rawFile.send(null);
}


function reorderWaypoints()
{
    viewer.entities.removeAll();

    for (n = 0; n < waypoints.length; n++)
    {
	if (n == 0)
	{
	    var redDot = viewer.entities.add({
		    position: Cesium.Cartesian3.fromDegrees(waypoints[n].longitude, waypoints[n].latitude, 150000.0),
        	    name : 'Current Location' + n,
		    id : n,
		    description : waypoints[n].latitude + ', ' + waypoints[n].longitude,
        	    ellipse : {
        	        semiMinorAxis : 1.0,
        	        semiMajorAxis : 1.0,
        	        height: 0.0,
        	        material : Cesium.Color.RED.withAlpha(0.5)
        	    },
		    waypoint : waypoints[n]
    	    });
	}
	else
	{
	    var blueDot = viewer.entities.add({
		    position: Cesium.Cartesian3.fromDegrees(waypoints[n].longitude, waypoints[n].latitude, 150000.0),
        	    name : 'Waypoint #' + n,
		    id : n,
		    description : waypoints[n].latitude + ', ' + waypoints[n].longitude,
        	    ellipse : {
        	        semiMinorAxis : 1.0,
        	        semiMajorAxis : 1.0,
        	        height: 0.0,
        	        material : Cesium.Color.BLUE.withAlpha(0.5)
        	    },
		    waypoint : waypoints[n]
    	    });
	}

	if (n > 0)
	{
	    var line = viewer.entities.add({
        	    name : 'Polyline',
		    id : n - 1 + '-' + n,
        	    polyline : {
                	        positions : Cesium.Cartesian3.fromDegreesArray([
                        	    waypoints[n-1].longitude, waypoints[n-1].latitude,
                            	    waypoints[n].longitude, waypoints[n].latitude
	                        ]),
        	                material : Cesium.Color.BLUE.withAlpha(0.5)
                	    }
	    });
	}
    }
    
    numPoints = waypoints.length;
    prevWaypoint.latitude = waypoints[waypoints.length - 1].latitude;
    prevWaypoint.longitude = waypoints[waypoints.length - 1].longitude;
}


function showWaypoints()
{
    for (n = 0; n < waypoints.length; n++)
	alert('Waypoint #'+ n + ': ' + waypoints[n].latitude + ', ' + waypoints[n].longitude);
}

function removeWaypoint(waypoint)
{
    var newWaypoints = [];
    var count = 0;
    for (n = 0; n < waypoints.length; n++)
    {
	if (waypoints[n].latitude == waypoint.latitude && waypoints[n].longitude == waypoint.longitude)
	{
//	    alert('Skipping ' + n + '... waypoint.latitude = ' + waypoint.latitude + ', waypoints[' + n +'].latitude = '+waypoints[n].latitude);
	}
	else
	{
//	    alert('Adding waypoint ' + n + '... waypoint.latitude = ' + waypoint.latitude + ', waypoints[n].latitude = ' + waypoints[n].latitude);
	    var newWaypoint = {
			latitude: waypoints[n].latitude,
			longitude: waypoints[n].longitude
	    };
	    newWaypoints.push(newWaypoint);
	    count++;
	}
    }

    waypoints = [];
    for (n = 0; n < newWaypoints.length; n++)
	waypoints.push(newWaypoints[n]);
}

function mouseClickHandler(e)
{
    var mousePosition = new Cesium.Cartesian2(e.clientX, e.clientY);

    var picked = pickEntity(viewer, mousePosition);
    if (picked != undefined)
    {
	if (picked.ellipse != undefined)
	{
	    viewer.entities.remove(picked);

	    var id = picked.id;
	    var prevId = id - 1;
	    var lineId = prevId + '-' + id;
	    viewer.entities.remove(viewer.entities.getById(lineId));

	    removeWaypoint(picked.waypoint);
	    reorderWaypoints();
	}
	return;
    }

    var ellipsoid = viewer.scene.globe.ellipsoid;
    var cartesian = viewer.camera.pickEllipsoid(mousePosition, ellipsoid);
    if (cartesian) {
        var cartographic = ellipsoid.cartesianToCartographic(cartesian);
	var waypoint = {
		latitude: Cesium.Math.toDegrees(cartographic.latitude).toFixed(6), 
		longitude: Cesium.Math.toDegrees(cartographic.longitude).toFixed(6)
	};


	var blueDot = viewer.entities.add({
		position: Cesium.Cartesian3.fromDegrees(waypoint.longitude, waypoint.latitude, 150000.0),
        	name : 'Waypoint #' + numPoints,
		id : numPoints,
		description : waypoint.latitude + ', ' + waypoint.longitude,
        	ellipse : {
        	    semiMinorAxis : 1.0,
        	    semiMajorAxis : 1.0,
        	    height: 0.0,
        	    material : Cesium.Color.BLUE.withAlpha(0.5)
        	},
		waypoint : waypoint
    	});

	var line = viewer.entities.add({
        	name : 'Polyline',
		id : numPoints - 1 + '-' + numPoints,
        	polyline : {
                	        positions : Cesium.Cartesian3.fromDegreesArray([
                        	    prevWaypoint.longitude, prevWaypoint.latitude,
                            	    waypoint.longitude, waypoint.latitude
	                        ]),
        	                material : Cesium.Color.BLUE.withAlpha(0.5)
                	    }
	});

	prevWaypoint.latitude = waypoint.latitude;
	prevWaypoint.longitude = waypoint.longitude;

	waypoints.push(waypoint);
	numPoints++;
    }
    else
    {
        alert('Globe was not picked');
    }	
}


/**
 * Returns the top-most Entity at the provided window coordinates
 * or undefined if no Entity is at that location.
 *
 * @param {Cartesian2} windowPosition The window coordinates.
 * @returns {Entity} The picked Entity or undefined.
 */
function pickEntity(viewer, windowPosition)
{
  var picked = viewer.scene.pick(windowPosition);
  if (picked != undefined) {
    var id = Cesium.defaultValue(picked.id, picked.primitive.id);
    if (id instanceof Cesium.Entity) {
      return id;
    }
  }
  return undefined;
};

function clearButtonClicked()
{
    var waypoints = [];
    waypoints.push(startingPoint);
    
    numPoints = waypoints.length;
    prevWaypoint.latitude = waypoints[waypoints.length - 1].latitude;
    prevWaypoint.longitude = waypoints[waypoints.length - 1].longitude;    

    viewer.entities.removeAll();
    var myLocation = viewer.entities.add({
            position: Cesium.Cartesian3.fromDegrees(prevWaypoint.longitude, prevWaypoint.latitude, 150000.0),
            name : 'Current Location',
            id : 0,
            ellipse : {
                semiMinorAxis : 1.0,
                semiMajorAxis : 1.0,
                height: 0.0,
                material : Cesium.Color.RED.withAlpha(0.5)
            },
	    waypoint : waypoints[0]
    });
}

function saveButtonClicked(frm)
{
    for (var i = 1; i < waypoints.length; i++)
    {
       var newHidInp = document.createElement('input');
           newHidInp.type  = 'hidden';
           newHidInp.name  = 'outArray[]';
           newHidInp.value = waypoints[i].latitude + '|' + waypoints[i].longitude;
       frm.appendChild(newHidInp);
    }
}

