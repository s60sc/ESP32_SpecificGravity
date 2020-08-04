
// populate datapoints
// Tilt Angle  Specific Gravity
const char* angle_gravity = R"~(
  25.0  1.002
  27.0  1.008
  28.5  1.012
  31.0  1.018
  33.5  1.022
  36.0  1.026
  38.0  1.028
  40.0  1.030
  43.0  1.034
  45.0  1.038
  61.3  1.071
)~";

// web page content
const char* index_html = R"~(
<!doctype html>
<html>
  <head>
    <meta http-equiv="Content-Type" content="text/html; charset=utf-8" />
    <title>ESP32_SpecificGravity</title>
    <script src="https://ajax.googleapis.com/ajax/libs/jquery/2.1.3/jquery.min.js"></script>
    <STYLE type="text/css">
      body {
        margin: 10px;
      }
    
      table {
        table-layout: fixed;
        font-family: arial, sans-serif;
        border-collapse: collapse;
        width: 100%;
      }
        
      td, th {
        border: 1px solid #dddddd;
        text-align: left;
        padding: 8px;
      }
      
      tr:nth-child(even) {
        background-color: #dddddd;
      } 
      input[type="text"] {
        font-size: 16px; 
        width: 100px
      }
      input[type="submit"], input[type="file"] {
        padding: 10px; 
        background-color: #e7e7e7;
        font-size: 16px; 
      }
    </STYLE>
  </head>
  <body>
    <table><tr>
      <th>Tilt Angle</th>
      <th>Specific Gravity</th>
      <th>Temperature</th>
      <th>Voltage</th>
      </tr><tr>
      <td id="1">Waiting</td>
      <td id="2">Waiting</td>
      <td id="3">Waiting</td>
      <td id="4">Waiting</td>
      </tr><tr>
      <td><input type="submit" id="StartBtn" value="Start"/></td>
      <td><input type="submit" id="ResetBtn" value="Reset"/></td>
      <form method='POST' action='/doOta' enctype='multipart/form-data' id='webOTA'>
        <td><input type='submit' value='OTA Update' id="OTAsubmit"></td>
        <td><input type='file' name='update'></td>
      </form></tr>
    </table>
    <script>
      var baseHost = document.location.origin
      var otaUrl = baseHost + '/update'
      var myUrl = baseHost +"/";
      var refreshRate = 2000; // 2 seconds
      $(function(){updatePage();}); 
    
      function updatePage(){ 
        console.log("updatePage");   
        // periodically update page content using received JSON
        var myData = $.ajax({ // send request to app
          url: myUrl+"refresh",           
          dataType : "json", 
          timeout : refreshRate, 
          success: function(data) { // receive response from app
            $.each(data, function(key, val) { 
              // replace each existing value with new value, using key name to match html tag id
              $('#'+key).text(val);
              $('#'+key).val(val);
            });
          }
        });
            
        myData.error(function(xhr, status, errorThrown){ 
          console.log("Failed to get data: " + errorThrown); 
          console.log("Status: " + status);
          console.dir(xhr);
        });
        timeOut = setTimeout('updatePage()', refreshRate);  // re-request data at refreshRate interval in ms
      }

      // show ota progress on new tab 
      $('#OTAsubmit').click(function() {
        window.open(baseHost +"/ota"); 
      });
      
      $('#StartBtn').click(function() {
        $.ajax({url: myUrl+"start"});
      });
      $('#ResetBtn').click(function() {
        $.ajax({url: myUrl+"reset"});
      });
    </script>
  </body>
</html>
)~";

const char* otaStatus = R"~(
<!--
  FSWebServer - Example Index Page
  Copyright (c) 2015 Hristo Gochkov. All rights reserved.
  This file is part of the ESP8266WebServer library for Arduino environment.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
-->
<!DOCTYPE html>
<html>
  <head>
    <meta http-equiv="Content-type" content="text/html; charset=utf-8">
    <title>OTA Monitor</title>
    <style type="text/css" media="screen">
    body {
      margin:0;
      padding:0;
      background-color: black;
    }

    #dbg, #input_div, #input_el {
      font-family: monaco;
      font-size: 12px;
      line-height: 13px;
      color: #AAA;
    }

    #dbg, #input_div {
      margin:0;
      padding:0;
      padding-left:4px;
    }

    #input_el {
      width:98%;
      background-color: rgba(0,0,0,0);
      border: 0px;
    }
    #input_el:focus {
      outline: none;
    }
    </style>
    <script type="text/javascript">
    var ws = null;
    function ge(s){ return document.getElementById(s);}
    function ce(s){ return document.createElement(s);}
    function stb(){ window.scrollTo(0, document.body.scrollHeight || document.documentElement.scrollHeight); }
    function sendBlob(str){
      var buf = new Uint8Array(str.length);
      for (var i = 0; i < str.length; ++i) buf[i] = str.charCodeAt(i);
      ws.send(buf);
    }
    function addMessage(m){
      var msg = ce("div");
      msg.innerText = m;
      ge("dbg").appendChild(msg);
      stb();
    }
    function startSocket(){
      ws = new WebSocket('ws://'+document.location.host+'/ws',['arduino']);
      ws.binaryType = "arraybuffer";
      ws.onopen = function(e){
        addMessage("Connected");
      };
      ws.onclose = function(e){
        addMessage("Disconnected");
      };
      ws.onerror = function(e){
        console.log("ws error", e);
        addMessage("Error");
      };
      ws.onmessage = function(e){
        var msg = "";
        if(e.data instanceof ArrayBuffer){
          msg = "BIN:";
          var bytes = new Uint8Array(e.data);
          for (var i = 0; i < bytes.length; i++) {
            msg += String.fromCharCode(bytes[i]);
          }
        } else {
          msg = "TXT:"+e.data;
        }
        addMessage(msg);
      };
      ge("input_el").onkeydown = function(e){
        stb();
        if(e.keyCode == 13 && ge("input_el").value != ""){
          ws.send(ge("input_el").value);
          ge("input_el").value = "";
        }
      }
    }
    function startEvents(){
      var es = new EventSource('/events');
      es.addEventListener('ota', function(e) {
        addMessage("OTA: " + e.data);
      }, false);
    }
    function onBodyLoad(){
      startSocket();
      startEvents();
    }
    </script>
  </head>
  <body id="body" onload="onBodyLoad()">
    <pre id="dbg"></pre>
    <div id="input_div">
      $<input type="text" value="" id="input_el">
    </div>
  </body>
</html>
)~";
