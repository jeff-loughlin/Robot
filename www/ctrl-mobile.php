<!DOCTYPE html>
<?php
  define('BASE_DIR', dirname(__FILE__));
  require_once(BASE_DIR.'/config.php');
?>
<html>
<head>
    <meta name="viewport" content="width=850px">
    <script src="script-robot.js"></script>

  <style>
     #dot{
            position: absolute;
            width: 5px;
            height: 5px;
            background-color: red;
	    border-radius: 50%;
	}
     #pantiltPtr{
            position: absolute;
            width: 16px;
            height: 16px;
            background-color: red;
	    border-radius: 50%;
            -webkit-user-select: none;
            -moz-user-select: none;
            -o-user-select: none;
            -ms-user-select: none;
            -khtml-user-select: none;
            user-select: none;
            cursor: default;
          }
     img{
            -webkit-user-select: none;
            -moz-user-select: none;
            -o-user-select: none;
            -ms-user-select: none;
            -khtml-user-select: none;
            user-select: none;
	    user-drag: none;
            cursor: default;
          }
    #pantilt{
            -webkit-user-select: none;
            -moz-user-select: none;
            -o-user-select: none;
            -ms-user-select: none;
            -khtml-user-select: none;
            user-select: none;
	    user-drag: none;
            cursor: default;
	}
    #ptDiv{
            -webkit-user-select: none;
            -moz-user-select: none;
            -o-user-select: none;
            -ms-user-select: none;
            -khtml-user-select: none;
            user-select: none;
	    user-drag: none;
            cursor: default;
	}

  </style>
  <script>

function motorTouchMove(event)
{
    event.preventDefault();
    motorMouseMove(event);
}

function motorTouchStart(event)
{
    event.preventDefault();
    motorMouseDown(event);
}

function motorTouchEnd(event)
{
    event.preventDefault();
    motorMouseUp();
}

var motorControlActive = false;

function motorMouseDown(event)
{
    motorControlActive = true;
    x = 251 - event.layerX;
    y = 192 - event.layerY;

    if (x > 128) x = 128;
    if (x < -127) x = -127;
    if (y > 128) y = 128;
    if (y < -127) y = -127;
    callPHP_motors('x=' + x + '&y=' + y);

    wallFollowModeLeft = false;
    wallFollowModeRight = false;
}

function motorMouseMove(event)
{
    if (motorControlActive)
    {
	x = 251 - event.layerX; //clientX;
	y = 192 - event.layerY; //clientY;
	if (x > 128) x = 128;
	if (x < -127) x = -127;
	if (y > 128) y = 128;
	if (y < -127) y = -127;
        callPHP_motors('x=' + x + '&y=' + y);
    }
}

function motorMouseUp()
{
    motorControlActive = false;
    callPHP_motors('x=0&y=0');
}

var panTiltGrabbed = false;

function touchMove(event)
{
    event.preventDefault();
    movePanTilt(event);
}

function touchStart(event)
{
    event.preventDefault();
    grabPanTilt(event);
}

function touchEnd(event)
{
    releasePanTilt();
}

function grabPanTilt(evt)
{
    panTiltGrabbed = true;
    var el = document.getElementById('pantiltPtr');
    el.style.top = evt.layerY - 7 + 'px';
    el.style.left = evt.layerX - 7 + 'px';

    x = evt.layerX - 7;
    y = evt.layerY - 7;
    x = x - 142;
    y = y - 142;
    y = -y;
    callPHP_PanTilt('x=' + x + '&y=' + y);
}

function movePanTilt(evt)
{
    if (panTiltGrabbed)
    {
	var el = document.getElementById('pantiltPtr');
	el.style.top = evt.layerY - 7 + 'px';
	el.style.left = evt.layerX - 7 + 'px';

        x = evt.layerX - 7;
        y = evt.layerY - 7;
        x = x - 142;
        y = y - 142;
	y = -y;
        callPHP_PanTilt('x=' + x + '&y=' + y);
    }
}

function releasePanTilt()
{
    panTiltGrabbed = false;
}

  </script>

</head>
<body onload="initialize();">


<!-- Video feed -->
<img style="position:absolute; top:10px; left:10px;" id="mjpeg_dest">
<div style="position:absolute;top:350px; left:15px; z-index:1000">
<input type="image" onclick="rotateLeft()" src="left-arrow.png" id="rotateLeftImage" style="height:30px;width:30px" />
</div>
<div style="position:absolute;top:350px; left:485px; z-index:1000">
<input type="image" onclick="rotateRight()" src="right-arrow.png" id="rotateRightImage" style="height:30px;width:30px" />
</div>
<!-- Pan/Tilt servo control element (crosshairs) -->
<div style="position:absolute; top:10px; left:550px">
<div id='pantiltPtr' style="position:absolute; top:142px; left:142px;" ></div>
<img draggable="false" style="width:300px; height:300px; opacity: 0.5;" src="crosshairs-400x400.png" id="pantilt" onmousedown='grabPanTilt(event);' onmousemove='movePanTilt(event);' onmouseup='releasePanTilt();' onmouseout='releasePanTilt();' ondragstart='return false;' ontouchstart='touchStart(event);' ontouchmove='touchMove(event);' ontouchend='touchEnd(event);' ontouchcancel='touchEnd(event);' />
<div style="position:absolute; top:10px; left:0px; font-size:8pt;">Pan/Tilt:</div>
<div id="panTiltText" style="position:absolute; top:0px; left:45px; font-size:8pt;"></div>
<div style="position:absolute; top: 320px; left:50px; width:35px"><button onclick='lookLeft()'>Left</button></div>
<div style="position:absolute; top: 320px; left:115px; width:35px"><button onclick='resetPanTilt()'>Center</button></div>
<div style="position:absolute; top: 320px; left:200px; width:35px"><button onclick='lookRight()'>Right</button></div>
</div>

<!-- Telemetry data superimposed over the cesium display and video feed -->
<img id="bearingArrow" src="pointer.png" style="position:absolute; top:35px; left: 20px; width:25px; height:25px"/>
<div id="headingIndicator" style="position:absolute; top: 35px;left:20px; color:red; font-size:8pt"></div>


<!-- Motor control element (crosshairs superimposed on video feed) -->
<img src="compass-640x480.png" id="compass" style="position:absolute; top:35px; left:15px; width:500px; height:375px"/>
<img onmousedown='motorMouseDown(event);' onmousemove='motorMouseMove(event);' onmouseup='motorMouseUp();' draggable="false" ondragstart="return false;" ontouchstart='motorTouchStart(event);' ontouchmove='motorTouchMove(event);' ontouchend='motorTouchEnd(event);' ontouchcancel='motorTouchEnd(event);'  src="crosshairs-640x480.png" style="opacity:0.5; position:absolute; top:35px; left:15px; height:375px; width:500px"/>

<!-- Comms Indicator -->
<div style="position:absolute; top:370px; left:550px;">Comm Status:</div>
<div id="commsIndicator" style="position:absolute; top:375px; left:645px; border-style:solid; border-width:1px; border-color:gray; border-radius:5px; width:9px; height:9px; background-color:red"></div>

<script type="text/javascript">

panCenter = 0;
tiltCenter = 0;
lookLeftPan = -60;
lookRightPan = 60;

function rotateLeft()
{
    sendCommand("ROT:-90");
}

function rotateRight()
{
    sendCommand("ROT:90");
}


function initialize()
{
    init();

    callPHP_getData();
    var ptr = document.getElementById("pantiltPtr");
    ptr.style.top = telemetry["Tilt Servo"] - 40 + 'px';
    ptr.style.left = telemetry["Pan Servo"] - 10 + 'px';

    setTimeout('initCamera();', 1000);
}

function initCamera()
{
    send_cmd('px 1296 976 25 25 2592 1944');
    setTimeout('initCamera2();', 500);
}

function initCamera2()
{
    send_cmd('sa 0');
    setTimeout('initCamera3();', 500);
}

function initCamera3()
{
    send_cmd('wb auto');
}

function resetPanTilt()
{
    sendCommand('PTT:'+panCenter+','+tiltCenter);
}

function lookLeft()
{
    sendCommand('PTT:'+lookLeftPan+','+tiltCenter);
}

function lookRight()
{
    sendCommand('PTT:'+lookRightPan+','+tiltCenter);
}

function callPHP_getData()
{
    var httpc = new XMLHttpRequest();
    var url = "getdata.php";
    httpc.open("GET", url, true);

    httpc.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");

    httpc.onreadystatechange = function() { //Call a function when the state changes.
        if(httpc.readyState == 4 && httpc.status == 200)
        {
            // complete and no errors
//	    if (httpc.responseText.length > 0)
//	        document.getElementById('data').innerHTML = httpc.responseText;
	    if (httpc.responseText.length > 0)
		formatTelemetryData(httpc.responseText);
        }
    }
    httpc.send("");
}

function sendCommand(cmd)
{
    var req = new XMLHttpRequest();
    req.open("POST","sendcommand.php", true);
    req.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");
    req.setRequestHeader("Content-Length", cmd.length); // POST request MUST have a Content-Length header (as per HTTP/1.1)
    req.onreadystatechange = function() {
        if (req.readyState == 4)
	{
	    if (req.status == 200)
	    {
//		alert(req.responseText);
	    }
	}
    }
    req.send("cmd=" + cmd);
}

var telemetry = new Array();
var updateBearingCounter = 0;
showCompass = true;

var timestampDupCount = 0;
var oldTimestamp;

function formatTelemetryData(data)
{
    var lines = data.split('\n');
    telemetry = [];
    for (n = 0; n < lines.length; n++)
    {
	var line = lines[n].split(":");
	if (line.length == 2)
	    telemetry[line[0]] = line[1].trim();
    }


    document.getElementById('headingIndicator').innerHTML = "Heading: " + telemetry["Heading"] + "<br/>Target Heading: " + telemetry["Target Heading"];

    if (showCompass == true)
    {
	var compassDir = 360 - telemetry["Heading"];

	document.getElementById('compass').style["webkitTransform"] = "rotate(" + compassDir + "deg)";
	document.getElementById('compass').style["MozTransform"] = "rotate(" + compassDir + "deg)";

	updateBearingCounter = 0;

	// Now calculate the bearing to the Set Heading so we can display an idicator for the direction we should be going
	var relativeBearing = telemetry["Target Heading"] - telemetry["Heading"];
	var bearingInRadians = relativeBearing * 3.14159265 / 180;

	// Rotate 90 degrees counterclockwise so the current heading is at the top
	var angle = bearingInRadians - 3.14159265 / 2;

	// Now angle represents a clockwise rotation from 0 (top) to 2PI.  Find the corresponding X and Y coordinates on a unit circle
	var x = Math.cos(angle);
	var y = Math.sin(angle);
	var h = 80; //Math.sqrt(250 * 250 + 200 * 200);

	// Translate the unit circle coordinates to screen coordinates to project a pointer onto the camera display at a position that
	// indicates the bearing to the set heading relative to the current heading
	x = 3 + 250 + x * h; // half the width of the video feed
	y = 10 + 200 + y * h; // half the height of the video feed


	// Display a pointer at the calculated coordinates
	document.getElementById('bearingArrow').style["webkitTransform"] = "rotate(" + relativeBearing + "deg)";
	document.getElementById('bearingArrow').style["MozTransform"] = "rotate(" + relativeBearing + "deg)";
	document.getElementById('bearingArrow').style.top = y + "px";
	document.getElementById('bearingArrow').style.left = x + "px";
    }



    // Update comms indicator
    timestamp = telemetry["Timestamp"];
    if (timestamp == oldTimestamp)
	timestampDupCount++;
    else
	timestampDupCount = 0;
    if (timestampDupCount > 10)
    {
	    document.getElementById('commsIndicator').style.backgroundColor = "red";
    }
    else
    {
	if (telemetry["LED"] == "ON")
	    document.getElementById('commsIndicator').style.backgroundColor = "green";
	else
	    document.getElementById('commsIndicator').style.backgroundColor = "white";
    }
    oldTimestamp = timestamp;


    // Update the Pan/Tilt Indicator
    document.getElementById('panTiltText').innerText = telemetry["Pan Servo"] + " / " + telemetry["Tilt Servo"];
    if (!panTiltGrabbed)
    {
	var ptr = document.getElementById("pantiltPtr");
	ptr.style.top = (-1 * telemetry["Tilt Servo"]) + 142 + 'px';
	ptr.style.left = (parseInt(telemetry["Pan Servo"]) + 142) + 'px';
    }
}



/*
 Here add the ID of the HTML elements for which to show the mouse coords
 Within quotes, separated by comma.
 E.g.:   ['imgid', 'divid'];
*/
var elmids = ['pantilt','motors'];

var x, y = 0;       // variables that will contain the coordinates

// Get X and Y position of the elm (from: vishalsays.wordpress.com)
function getXYpos(elm)
{
  x = elm.offsetLeft;        // set x to elm's offsetLeft
  y = elm.offsetTop;         // set y to elm's offsetTop

  elm = elm.offsetParent;    // set elm to its offsetParent

  //use while loop to check if elm is null
  // if not then add current elm's offsetLeft to x
  //offsetTop to y and set elm to its offsetParent
  while(elm != null)
  {
    x = parseInt(x) + parseInt(elm.offsetLeft);
    y = parseInt(y) + parseInt(elm.offsetTop);
    elm = elm.offsetParent;
  }

  // returns an object with "xp" (Left), "=yp" (Top) position
  return {'xp':x, 'yp':y};
}




function callPHP_PanTilt(params)
{
    var httpc = new XMLHttpRequest();
    var url = "pantilt.php";
    httpc.open("POST", url, true);

    httpc.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");
    httpc.setRequestHeader("Content-Length", params.length); // POST request MUST have a Content-Length header (as per HTTP/1.1)

    httpc.onreadystatechange = function() { //Call a function when the state changes.
        if (httpc.readyState == 4 && httpc.status == 200)
        {
            // complete and no errors
//	    document.getElementById('coordsPanTilt').innerHTML = "Pan/Tilt Cmd: " + httpc.responseText;
        }
    }
    httpc.send(params);
}

function callPHP_motors(params)
{
    var httpc = new XMLHttpRequest();
    var url = "motorcontrol.php";
    httpc.open("POST", url, true);

    httpc.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");
    httpc.setRequestHeader("Content-Length", params.length); // POST request MUST have a Content-Length header (as per HTTP/1.1)

    httpc.onreadystatechange = function() { //Call a function when the state changes.
        if(httpc.readyState == 4 && httpc.status == 200)
        {
	    // complete and no errors
	    document.getElementById('coordsMotors').innerHTML = "Motors Cmd: " + httpc.responseText;
        }
        else if (httpc.readyState == 4)
        {
            alert("Something horrible happened: " + httpc.status);
        }
    }
    httpc.send(params);
}


var ptActive = false;
var mActive = false;

// Get X, Y coords, and displays Mouse coordinates
function getCoordsPanTilt(e)
{
   // coursesweb.net/
   var xy_pos = getXYpos(this);

   // if IE
   if(navigator.appVersion.indexOf("MSIE") != -1)
   {
      // in IE scrolling page affects mouse coordinates into an element
      // This gets the page element that will be used to add scrolling value to correct mouse coords
      var standardBody = (document.compatMode == 'CSS1Compat') ? document.documentElement : document.body;

      x = event.clientX + standardBody.scrollLeft;
      y = event.clientY + standardBody.scrollTop;
   }
   else
   {
      x = e.pageX;
      y = e.pageY;
   }

   x = x - xy_pos['xp'] - 250;
   y = 303 - y - xy_pos['yp'];

   // displays x and y coords in the #coords element
//  document.getElementById('coords').innerHTML = 'X= '+ x+ ' ,Y= ' +y;

   if (ptActive)
   {
      callPHP_PanTilt('x=' + x + '&y=' + y);
   }
}

var timer = setInterval(function() { callPHP_getData() }, 100);


</script>

</body>
</html>
