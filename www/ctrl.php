<!DOCTYPE html>
<?php
  define('BASE_DIR', dirname(__FILE__));
  require_once(BASE_DIR.'/config.php');
?>
<html>
<head>
    <link rel="stylesheet" type="text/css" href="css/flightindicators.css" />
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
//            position: absolute;
//            width: 10px;
//            height: 10px;
//            background-color: blue;
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
.unselectable {
    /* For Opera and <= IE9, we need to add unselectable="on" attribute onto each element */
    /* Check this site for more details: http://help.dottoro.com/lhwdpnva.php */
    -moz-user-select: none; /* These user-select properties are inheritable, used to prevent text selection */
    -webkit-user-select: none;
    -ms-user-select: none; /* From IE10 only */
    user-select: none; /* Not valid CSS yet, as of July 2012 */

    -webkit-user-drag: none; /* Prevents dragging of images/divs etc */
    user-drag: none;
}

  </style>
  <script>
var makeUnselectable = function( $target ) {
    $target
        .addClass( 'unselectable' ) // All these attributes are inheritable
        .attr( 'unselectable', 'on' ) // For IE9 - This property is not inherited, needs to be placed onto everything
        .attr( 'draggable', 'false' ) // For moz and webkit, although Firefox 16 ignores this when -moz-user-select: none; is set, it's like these properties are mutually exclusive, seems to be a bug.
        .on( 'dragstart', function() { return false; } );  // Needed since Firefox 16 seems to ingore the 'draggable' attribute we just applied above when '-moz-user-select: none' is applied to the CSS 

    $target // Apply non-inheritable properties to the child elements
        .find( '*' )
        .attr( 'draggable', 'false' )
        .attr( 'unselectable', 'on' ); 
};

     function $(el)
     {
         return document.getElementById(el);
     }
     var tzdragg = function()
                   {
                       return {
catchup: function(evt)
{
el = document.getElementById("pantiltPtr");
el.style.top = evt.layerY + 'px';
el.style.left = evt.layerX + 'px';
},

                                  move : function(divid,xpos,ypos)
                                         {
                                             var a = $(divid);
                                             $(divid).style.left = xpos + 'px';
                                             $(divid).style.top = ypos + 'px';

                                             x = xpos;
                                             y = ypos;
                                             if (divid == "pantiltPtr")
                                             {
                                                 x = x + 10;
                                                 y = 140 - y;

                                                 callPHP_PanTilt('x=' + x + '&y=' + y);
                                             }
                                         },

                                  startMoving : function(evt)
                                                {
                                                    evt = evt || window.event;
                                                    var posX = evt.clientX,
                                                        posY = evt.clientY,
                                                        a = $(evt.target.id),
                                                        divTop = a.style.top,
                                                        divLeft = a.style.left;
                                                        divTop = divTop.replace('px','');
                                                        divLeft = divLeft.replace('px','');
                                                    var diffX = posX - divLeft,
                                                        diffY = posY - divTop;

                                                    document.onmousemove = function(evt)
                                                                           {
                                                                               evt = evt || window.event;
                                                                               var posX = evt.clientX,
                                                                                   posY = evt.clientY,
                                                                                   aX = posX - diffX,
                                                                                   aY = posY - diffY;

                                                                               if (evt.target.id == 'pantiltPtr')
                                                                               {
                                                                                   if (aX < 200 && aX > 0 && aY < 200 && aY > 0)
                                                                                       tzdragg.move(evt.target.id,aX,aY);
                                                                                   else
                                                                                       tzdragg.stopMoving(evt.target.id);
                                                                               }
                                                                           }
                                                },
                                  stopMoving : function(divid)
                                               {
                                                   var a = document.createElement('script');
                                                   document.onmousemove = function() {}
                                               },
                              }
                    }();

var motorControlActive = false;
function motorMouseDown()
{
    motorControlActive = true;
//    x = event.clientX - 266;  // Swap L/R motors
    x = 266 - event.clientX;
    y = 222 - event.clientY;

    if (x > 128) x = 128;
    if (x < -127) x = -127;
    if (y > 128) y = 128;
    if (y < -127) y = -127;
//    document.getElementById('coords').innerHTML = "Motors: x=" + x + ", y=" + y;
    callPHP_motors('x=' + x + '&y=' + y);
}

function motorMouseMove(event)
{
    if (motorControlActive)
    {
//	x = event.clientX - 266;  // Swap L/R motors
	x = 266 - event.clientX;
	y = 222 - event.clientY;
	if (x > 128) x = 128;
	if (x < -127) x = -127;
	if (y > 128) y = 128;
	if (y < -127) y = -127;
        callPHP_motors('x=' + x + '&y=' + y);

//        document.getElementById('coords').innerHTML = 'Motors: x=' + x + ', y=' + y;
    }
}

function motorMouseUp()
{
    motorControlActive = false;
    callPHP_motors('x=0&y=0');
}

var panTiltGrabbed = false;

function grabPanTilt(evt)
{
    panTiltGrabbed = true;
    var el = document.getElementById('pantiltPtr');
    el.style.top = evt.layerY - 7 + 'px';
    el.style.left = evt.layerX - 7 + 'px';

    x = evt.layerX - 7;
    y = evt.layerY - 7;
    x = x + 10;
    y = 140 - y;
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
        x = x + 10;
        y = 140 - y;
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
<script src="http://ajax.googleapis.com/ajax/libs/jquery/2.1.0/jquery.min.js"></script>
<script src="js/jquery.flightindicators.js"></script>
<br/>

<!-- Video control buttons -->
<div style="position:absolute; top:0px; left:10px; z-index:100">
      <input id="video_button" type="button">
      <input id="image_button" type="button">
      <input id="timelapse_button" type="button">
      <input id="md_button" type="button">
      <input id="halt_button" type="button">
</div>


<!-- Video feed -->
<img style="position:absolute; top:30px; left:10px;" id="mjpeg_dest">
<div style="position:absolute; top:415px; left:325px; z-index:1000">
White Balance:
<select id="wbSelector" onclick="send_cmd('wb ' + this.value)">
<?php
   $options_wb = array('Off' => 'off', 'Auto' => 'auto', 'Sun' => 'sun', 'Cloudy' => 'cloudy', 'Shade' => 'shade', 'Tungsten' => 'tungsten', 'Fluorescent' => 'fluorescent', 'Incandescent' => 'incandescent', 'Flash' => 'flash', 'Horizon' => 'horizon');
      foreach($options_wb as $name => $value) {
         if ($cvalue != $value) {
            $selected = '';
         } else {
            $selected = ' selected';
         }
         echo "<option value='$value'$selected>$name</option>";
      }
?>
</select><br/>
Grayscale: <input id="grayscaleCheckbox" style="vertical-align:bottom" type="checkbox" onclick="if (this.checked) send_cmd('sa -100'); else send_cmd('sa 0');"/>
</div>
<div style="position:absolute;top:415px; left:210px; z-index:1000">
<input type="button" onclick="showHideCompass()" value="Hide Compass" id="compassButton"/>
</div>
<div style="position:absolute;top:380px; left:15px; z-index:1000">
<input type="image" onclick="rotateLeft()" src="left-arrow.png" id="rotateLeftImage" style="height:30px;width:30px" />
</div>
<div style="position:absolute;top:380px; left:485px; z-index:1000">
<input type="image" onclick="rotateRight()" src="right-arrow.png" id="rotateRightImage" style="height:30px;width:30px" />
</div>
<!-- Pan/Tilt servo control element (crosshairs) -->
<div style="position:absolute; top:30px; left:550px">
<div id='pantiltPtr' style="top:92px; left:92px;" ></div>
<img draggable="false" style="width:200px; height:200px; opacity: 0.5;" src="crosshairs-400x400.png" id="pantilt" onmousedown='grabPanTilt(event);' onmousemove='movePanTilt(event);' onmouseup='releasePanTilt();' onmouseout='releasePanTilt();' ondragstart='return false;'/>
<div style="position:absolute; top:0px; left:0px">Pan/Tilt</div>
<div style="position:absolute; top: 220px; left:0px; width:35px"><button onclick='lookLeft()'>Look Left</button></div>
<div style="position:absolute; top: 220px; left:65px; width:35px"><button onclick='resetPanTilt()'>Center Camera</button></div>
<div style="position:absolute; top: 220px; left:150px; width:35px"><button onclick='lookRight()'>Look Right</button></div>
</div>


<!-- Motor control element (crosshairs superimposed on video feed) -->
<img src="compass-640x480.png" id="compass" style="position:absolute; top:35px; left:15px; width:500px; height:375px"/>
<img onmousedown='motorMouseDown();' onmousemove='motorMouseMove(event);' onmouseup='motorMouseUp();' draggable="false" ondragstart="return false;" src="crosshairs-640x480.png" style="opacity:0.5; position:absolute; top:35px; left:15px; height:375px; width:500px"/>

<!-- Telemetry data -->
<div style="position:absolute; top:620px; left:10px; " id="data"></div>


<!-- Misc informational text - motor command, download linkm mouse coords (for debugging), etc -->
<div style="position:absolute; top: 400px; left:10px"><p><a href="vids.php">Download Videos and Images</a></p></div>
<div style="position: absolute; top: 600px; left: 10px" id="coordsMotors">Motors Cmd:</div>
<div style="position:absolute; left:550px; top:300px;" id="coords"></div>


<!-- Cesium display for GPS and waypoint controls -->
<div style="position:absolute; top:30px; left:775px">
<iframe id="cesiumFrame" width="525" height="380" src="cesium/Apps/position.html" onload="function() {this.contentWindow.location.reload(); this.onload = null;}"></iframe>
</div>

<div style="position:absolute; top:420px; left:775px">
<input type="button" onclick="location.href='cesium/Apps/Waypoints/waypoints.html';" value="Define Waypoints" />
<input type="button" onclick="prevWaypoint();" value="<" />
<input type="button" onclick="nextWaypoint();" value=">" />
<input type="button" onclick="resetGPSTrack();" value="Reset" />
</div>

<!-- Telemetry data superimposed over the cesium display and video feed -->
<div id="latLong" style="position:absolute; top:35px; left:785px; color:#ff0000"></div>
<img id="bearingArrow" src="pointer.png" style="position:absolute; top:35px; left: 20px; width:25px; height:25px"/>
<div id="headingIndicator" style="position:absolute; top: 35px;left:20px; color:#ff0000"></div>


<!-- Speak input box -->
<div style="position:absolute; top:470px; left:420px; border-style:solid; border-width:1px; border-color:#c0c0c0; border-radius:5px">
<br/>
&nbsp;<input type="text" size="60" id="speakText">
<input type="button" value="Speak" onclick="speakButtonClicked()"/>&nbsp;<br/><br/>
&nbsp;Volume: <input type="range" id="volRange" style="position:absolute; top:67px; left:55px" min="75" max="100" value="50" onchange="volumeChanged()"/><br/>
&nbsp;Speed: <input type="range" id="speedRange" style="position:absolute; top:87px; left:55px" min="50" max="255" value="110"/>
<br/>
&nbsp;
</div>


<div style="position:absolute; top:620px; left:490px; border-style:solid; border-width:1px; border-color:#c0c0c0; border-radius:5px">

<table>
  <tr>
    <td><input id="laserButton" type="button" value="Laser On" onClick="laserButtonClicked()" style="width:150px"/></td>
    <td><input id="catButton" type="button" value="Cat-toy Mode On" onClick="catButtonClicked()" style="width:150px"/></td>
  </tr>

  <tr>
    <td><input id="scanButton" type="button" value="Scan Mode On" onClick="scanButtonClicked()" style="width:150px"/></td>
    <td><input id="motionDetectButton" type="button" value="Motion Detection On" onClick="motionDetectButtonClicked()" style="width:150px"/></td>
  </tr>

  <tr>
    <td><input id="leftWallButton" type="button" value="Wall Follow (Left)" onClick="leftWallButtonClicked()" style="width:150px"/></td>
    <td><input id="rightButton" type="button" value="Wall Follow (Right)" onClick="rightWallButtonClicked()" style="width:150px"/></td>
  </tr>

  <tr>
    <td><input id="meowButton" type="button" value="Meow" onClick="meowButtonClicked()" style="width:150px"/></td>
    <td><input id="barkButton" type="button" value="Bark" onClick="barkButtonClicked()" style="width:150px"/></td>
  </tr>
</table>
</div>

<div style="position:absolute; top:770px; left:500px; border-style:solid; border-width:1px; border-color:#c0c0c0; border-radius:5px">
<table>
<tr><td align="right">Main Bus Voltage:</td><td><meter id="voltageMeter1" value="0" min="0" max="16.0" high="15.0" low="11.0" optimum="13.0"></meter></td><td align="right"><span id="voltageText1">0.00V</span></td></tr>
<tr><td align="right">Secondary Bus Voltage:</td><td><meter id="voltageMeter2" value="5" min="0" max="16.0" high="7.0" low="3.0"optimum="5.0"></meter></td><td align="right"><span id="voltageText2">5.0V</span></td></tr>
</table>
</div>

<div id="debug" style="position:absolute; top:420px; left:525px;"></div>

<!-- Battery and attitude indicators -->
<div style="position:absolute; top:300px; left:525px;">
<span id="attitude"></span>
<span id="battery"></span>
</div>


<!-- Gyro meters -->
<div style="width:70px; height:120px; position:absolute; top:470px; left:10px; border-style:solid; border-width:1px; border-color:#c0c0c0; border-radius:5px">
<div style="width:70px; text-align:center;">Gyros</div>
<span style="position:absolute; left:-25px; top:55px;"><meter style="width:80px; transform:rotate(-90deg); -webkit-transform: rotate(-90deg); -moz-transform: rotate(-90deg);" id="gyroXMeter" value="0" min="0.0" max="20.0" high="10" optimum="0"></span>
<span style="position:absolute; left:-5px; top:55px;"><meter style="width:80px; transform:rotate(-90deg); -webkit-transform: rotate(-90deg); -moz-transform: rotate(-90deg);" id="gyroYMeter" value="0" min="0.0" max="20.0" high="10" optimum="0"></span>
<span style="position:absolute; left:15px; top:55px;"><meter style="width:80px; transform:rotate(-90deg); -webkit-transform: rotate(-90deg); -moz-transform: rotate(-90deg);" id="gyroZMeter" value="0" min="0.0" max="20.0" high="10" optimum="0"></span>
<span style="position:absolute; left:12px; top:100px">x</span>
<span style="position:absolute; left:32px; top:100px">y</span>
<span style="position:absolute; left:52px; top:100px">z</span>
</div>

<!-- Motor meters -->
<div style="width:70px; height:120px; position:absolute; top:470px; left:150px; border-style:solid; border-width:1px; border-color:#c0c0c0; border-radius:5px">
<div style="width:70px; text-align:center;">Motors</div>
<span style="position:absolute; left:7px; top:33px;"><meter style="width:40px; transform:rotate(-90deg); -webkit-transform: rotate(-90deg); -moz-transform: rotate(-90deg);" id="leftMotorMeterPos" value="0" min="0.0" max="75.0"></span>
<span style="position:absolute; left:27px; top:33px;"><meter style="width:40px; transform:rotate(-90deg); -webkit-transform: rotate(-90deg); -moz-transform: rotate(-90deg);" id="rightMotorMeterPos" value="0" min="0.0" max="75.0"></span>
<span style="position:absolute; left:7px; top:73px;"><meter style="width:40px; transform:rotate(90deg) scaleY(-1); -webkit-transform: rotate(90deg) scaleY(-1); -moz-transform: rotate(90deg) scaleY(-1);" id="leftMotorMeterNeg" value="0" min="0" max="75" high="2" optimum="1"></span>
<span style="position:absolute; left:27px; top:73px;"><meter style="width:40px; transform:rotate(90deg) scaleY(-1); -webkit-transform: rotate(90deg) scaleY(-1); -moz-transform: rotate(90deg) scaleY(-1);" id="rightMotorMeterNeg" value="0" min="0.0" max="75.0" high="2" optimum="1"></span>
<span id="LValue" style="width:40px; text-align:center; position:absolute; left:8px; top:104px; font-size:8pt">L</span>
<span id="RValue" style="width:40px; text-align:center; position:absolute; left:27px; top:104px; font-size:8pt">R</span>
</div>


<script type="text/javascript">

panCenter = 102;
tiltCenter = 180-132;

lookLeftPan = 50;
lookRightPan = 150;

catMode = 0;
laserOn = 0;
scanMode = 0;
motionDetectMode = 0;

function scanButtonClicked()
{
    if (scanMode)
    {
        sendCommand("SCN:0");
	document.getElementById("scanButton").value="Scan Mode On";
	var ptr = document.getElementById("pantiltPtr");
	ptr.style.top = telemetry["Tilt Servo"] - 40 + 'px';
	ptr.style.left = telemetry["Pan Servo"] - 10 + 'px';
    }
    else
    {
	sendCommand("SCN:1");
	document.getElementById("scanButton").value="Scan Mode Off";
    }

    scanMode = !scanMode;
}

function catButtonClicked()
{
    if (catMode)
    {
        sendCommand("CAT:0");
	document.getElementById("catButton").value="Cat-toy Mode On";
	laserOn = 0;
    }
    else
    {
	sendCommand("CAT:1");
	document.getElementById("catButton").value="Cat-toy Mode Off";
	laserOn = 1;
    }

    catMode = !catMode;
}

function motionDetectButtonClicked()
{
    if (motionDetectMode)
    {
        sendCommand("MOT:0");
	document.getElementById("motionDetectButton").value="Motion Detection On";
    }
    else
    {
	sendCommand("MOT:1");
	document.getElementById("motionDetectButton").value="MotionDetection Off";
    }

    motionDetectMode = !motionDetectMode;
}

function laserButtonClicked()
{
    if (laserOn)
    {
        sendCommand("LAS:0");
	document.getElementById("laserButton").value="Laser On";
    }
    else
    {
	sendCommand("LAS:1");
	document.getElementById("laserButton").value="Laser Off";
    }

    laserOn = !laserOn;
}

function rotateLeft()
{
    sendCommand("ROT:-90");
}

function rotateRight()
{
    sendCommand("ROT:90");
}

function leftWallButtonClicked()
{
    sendCommand("W:L");
}

function rightWallButtonClicked()
{
    sendCommand("W:R");
}

function speakButtonClicked() {
    var text = document.getElementById("speakText").value;
    var req = new XMLHttpRequest();
    req.open("POST","speak.php", true);
    req.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");
    req.setRequestHeader("Content-Length", text.length); // POST request MUST have a Content-Length header (as per HTTP/1.1)
    req.onreadystatechange = function() {
        if (req.readyState == 4)
	{
	    if (req.status == 200)
	    {
//		alert(req.responseText);
	    }
	}
    }
    req.send("text=" + text + "&speed=" + speedRange.value);
//    document.getElementById("speakText").value = "";
    document.getElementById("speakText").focus();
}

function meowButtonClicked() {
    var req = new XMLHttpRequest();
    req.open("POST","meow.php", true);
    req.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");
    req.setRequestHeader("Content-Length", 0); // POST request MUST have a Content-Length header (as per HTTP/1.1)
    req.onreadystatechange = function() {
        if (req.readyState == 4)
	{
	    if (req.status == 200)
	    {
//		alert(req.responseText);
	    }
	}
    }
    req.send("");
}

function barkButtonClicked() {
    var req = new XMLHttpRequest();
    req.open("POST","bark.php", true);
    req.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");
    req.setRequestHeader("Content-Length", 0); // POST request MUST have a Content-Length header (as per HTTP/1.1)
    req.onreadystatechange = function() {
        if (req.readyState == 4)
	{
	    if (req.status == 200)
	    {
//		alert(req.responseText);
	    }
	}
    }
    req.send("");
}

function volumeChanged()
{
    var req = new XMLHttpRequest();
    req.open("POST","volume.php", true);
    req.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");
//    req.setRequestHeader("Content-Length", vol.length); // POST request MUST have a Content-Length header (as per HTTP/1.1)
    req.onreadystatechange = function() {
        if (req.readyState == 4)
	{
	    if (req.status == 200)
	    {
//		alert(req.responseText);
	    }
	}
    }
    req.send("vol=" + volRange.value);
}

function resetGPSTrack()
{
    sendCommand("RW:0");
}

function prevWaypoint()
{
    sendCommand("WPT:-1");
}

function nextWaypoint()
{
    sendCommand("WPT:1");
}

function initialize()
{
    init();
    cesiumFrame.contentWindow.location.reload();
    sendCommand("RW:0");
    document.getElementById("speedRange").value = 110;
    document.getElementById("volRange").value = 85;

    callPHP_getData();
    var ptr = document.getElementById("pantiltPtr");
    ptr.style.top = telemetry["Tilt Servo"] - 40 + 'px';
    ptr.style.left = telemetry["Pan Servo"] - 10 + 'px';
    document.getElementById("speakText").focus();

    document.getElementById("wbSelector").value="auto";
    document.getElementById("grayscaleCheckbox").checked = false;

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
    init_gauges();
}

var attitude = $.flightIndicator('#attitude', 'attitude', {size:125, roll:0, pitch:0, showBox : false});
var battery = $.flightIndicator('#battery', 'airspeed', {size:125, showBox : false});
function init_gauges()
{
}

function resetPanTilt()
{
    document.getElementById('pantiltPtr').style.top = "92px";
    document.getElementById('pantiltPtr').style.left = "92px";
    sendCommand('PT:'+panCenter+','+tiltCenter);
}

function lookLeft()
{
    document.getElementById('pantiltPtr').style.top = "92px";
    document.getElementById('pantiltPtr').style.left = "42px";
    sendCommand('PT:'+lookLeftPan+','+tiltCenter);
}

function lookRight()
{
    document.getElementById('pantiltPtr').style.top = "92px";
    document.getElementById('pantiltPtr').style.left = "142px";
    sendCommand('PT:'+lookRightPan+','+tiltCenter);
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

function showHideCompass()
{
    if (showCompass)
    {
	showCompass = false;
	document.getElementById("headingIndicator").style["visibility"] = "hidden";
	document.getElementById("bearingArrow").style["visibility"] = "hidden";
	document.getElementById("compass").style["visibility"] = "hidden";
	document.getElementById("compassButton").value = "Show Compass";
    }
    else
    {
	showCompass = true;
	document.getElementById("headingIndicator").style["visibility"] = "visible";
	document.getElementById("bearingArrow").style["visibility"] = "visible";
	document.getElementById("compass").style["visibility"] = "visible";
	document.getElementById("compassButton").value = "Hide Compass";
    }
}

function formatTelemetryData(data)
{
    var lines = data.split('\n');
    for (n = 0; n < lines.length; n++)
    {
	var line = lines[n].split(":");
	if (line.length == 2)
	    telemetry[line[0]] = line[1].trim();
    }

    document.getElementById('data').innerHTML = "";
    for (obj in telemetry)
	document.getElementById('data').innerHTML += obj + ": " + telemetry[obj] + "<br/>";

    document.getElementById('latLong').innerHTML = "Position: " + telemetry["Latitude"] + ", " + telemetry["Longitude"] +
		"<br/>Range: " + telemetry["Waypoint Range"] + "<br/>Heading: " + telemetry["Heading"];

    document.getElementById('headingIndicator').innerHTML = "Heading: " + telemetry["Heading"] + "<br/>Target Heading: " + telemetry["Target Heading"];

    if (showCompass == false)
        return;

//    document.getElementById('headingArrow').style.MozTransform = "rotate(" + telemetry["heading"] + "deg)";
//    document.getElementById('headingArrow').style.webkitTransform = "rotate(" + telemetry["heading"] + "deg)";
//    document.getElementById('headingArrow').style.khtmlTransform = "rotate(" + telemetry["heading"] + "deg)";
    var compassDir = 360 - telemetry["Heading"];
//    document.getElementById('headingArrow').style["webkitTransform"] = "rotate(" + compassDir + "deg)";
//    document.getElementById('headingArrow').style["MozTransform"] = "rotate(" + compassDir + "deg)";

    document.getElementById('compass').style["webkitTransform"] = "rotate(" + compassDir + "deg)";
    document.getElementById('compass').style["MozTransform"] = "rotate(" + compassDir + "deg)";


//    if (updateBearingCounter++ <= 10)
//	return;

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

    // Show some debugging info
//    document.getElementById('debug').innerHTML="relativeBearing = " + relativeBearing + "<br/>bearingInRadians = " + parseFloat(Math.round(bearingInRadians * 100) / 100).toFixed(2) + "<br/>angle = " + parseFloat(Math.round(angle * 100) / 100).toFixed(2) + "<br/>x = " + parseFloat(Math.round(x * 100) / 100).toFixed(2) + "<br/>y = " + parseFloat(Math.round(y * 100) / 100).toFixed(2);

    // Update the voltage meter
    document.getElementById('voltageMeter1').value = telemetry["Main Bus Voltage"].slice(0,-1);
    document.getElementById('voltageText1').innerText = telemetry["Main Bus Voltage"];

    // Update the gyro meters
    document.getElementById('gyroXMeter').value = Math.abs(telemetry["GyroX"]);
    document.getElementById('gyroYMeter').value = Math.abs(telemetry["GyroY"]);
    document.getElementById('gyroZMeter').value = Math.abs(telemetry["GyroZ"]);

    // Update the motor meters
    leftMotorPower = telemetry["LEFT MOTOR"].trim();
    rightMotorPower = telemetry["RIGHT MOTOR"].trim();
    if (leftMotorPower > 0)
    {
	document.getElementById('leftMotorMeterPos').value = leftMotorPower;
	document.getElementById('leftMotorMeterNeg').value = 0;
    }
    else
    {
	document.getElementById('leftMotorMeterNeg').value = -leftMotorPower;
	document.getElementById('leftMotorMeterPos').value = 0;
    }
    if (rightMotorPower > 0)
    {
	document.getElementById('rightMotorMeterPos').value = rightMotorPower;
	document.getElementById('rightMotorMeterNeg').value = 0;
    }
    else
    {
	document.getElementById('rightMotorMeterNeg').value = -rightMotorPower;
	document.getElementById('rightMotorMeterPos').value = 0;
    }
    document.getElementById('LValue').innerHTML=leftMotorPower;
    document.getElementById('RValue').innerHTML=rightMotorPower;


    // Update the attitude indicator
//    attitude.setRoll(telemetry["Roll Angle"].slice(1,-8));
//    attitude.setPitch(telemetry["Pitch Angle"].slice(1,-8));
    roll = parseFloat(telemetry["Roll Angle"].trim());
    pitch = parseFloat(telemetry["Pitch Angle"].trim());
    attitude.setRoll(roll);
    attitude.setPitch(pitch);

    // Update the battery indicator
    batt = parseFloat(telemetry["Main Bus Voltage"].trim());
    battery.setAirSpeed(batt * 10);
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
