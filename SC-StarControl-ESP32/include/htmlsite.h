#include <Arduino.h>

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>

<head>
    <title>Star Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <link rel="icon" href="data:,">
    <style>
        html {
            font-family: Arial;
            display: inline-block;
            text-align: center;
            background: #000000
        }

        h2 {
            font-size: 3.0rem;
            color: #ffffff
        }

        h4 {
            font-size: 1.75rem;
            color: #ffffff
        }

        p {
            font-size: 3.0rem;
            color: #ffffff
        }

        body {
            max-width: 600px;
            margin: 0px auto;
            padding-bottom: 25px;
        }

        input[type=text] {
            width: 40%;
            padding: 10px 14px;
            margin: 8px 0;
            display: inline-block;
            border: 1px solid #444;
            border-radius: 8px;
            box-sizing: border-box;
            background-color: #222;
            color: white;
            font-size: 1.25rem;
            text-align: center;
        }

        .switch {
            position: relative;
            display: inline-block;
            width: 200px;
            height: 40px
        }

        .switch input {
            display: none
        }

        .btnslider {
            position: absolute;
            top: 0;
            left: 0;
            right: 0;
            bottom: 0;
            background-color: #222;
            border-radius: 6px
        }

        .btnslider:before {
            position: absolute;
            content: "";
            height: 25px;
            width: 100px;
            left: 8px;
            bottom: 8px;
            background-color: #555;
            -webkit-transition: .4s;
            transition: .4s;
            border-radius: 3px
        }

        input:checked+.btnslider {
            background-color: #21a707
        }

        input:checked+.btnslider:before {
            -webkit-transform: translateX(84px);
            -ms-transform: translateX(84px);
            transform: translateX(84px)
        }

        .sliderMain {
            -webkit-appearance: none;
            margin: -40px;
            width: 275px;
            height: 25px;
            background: #222;
            outline: none;
            -webkit-transition: .2s;
            transition: opacity .2s;
            border-radius: 15px;
        }

        .sliderMain::-webkit-slider-thumb {
            -webkit-appearance: none;
            appearance: none;
            width: 35px;
            height: 35px;
            background: #555;
            border-radius: 25px;
            cursor: pointer;
        }

        .sliderMain::-moz-range-thumb {
            width: 35px;
            height: 35px;
            cursor: pointer;
        }

        .sliderMainText {
            font-size: 1.5rem;
            color: white;
        }

        .sliderFadeSpeed {
            -webkit-appearance: none;
            margin: -40px;
            width: 250px;
            height: 25px;
            background: #222;
            outline: none;
            -webkit-transition: .2s;
            transition: opacity .2s;
            border-radius: 15px;
        }

        .sliderFadeSpeed::-webkit-slider-thumb {
            -webkit-appearance: none;
            appearance: none;
            width: 35px;
            height: 35px;
            background: #555;
            border-radius: 25px;
            cursor: pointer;
        }

        .sliderFadeSpeed::-moz-range-thumb {
            width: 35px;
            height: 35px;
            cursor: pointer;
        }

        .sliderFadeSpeedTextStar {
            font-size: 1.5rem;
            color: white;
        }

        .sliderFadeSpeedTextTFL {
            font-size: 1.5rem;
            color: white;
        }

        .buttonMomentary {
            padding: 10px 20px;
            font-size: 24px;
            text-align: center;
            outline: none;
            color: #fff;
            background-color: #2f4468;
            border: none;
            border-radius: 5px;
            box-shadow: 0 6px #999;
            cursor: pointer;
            -webkit-touch-callout: none;
            -webkit-user-select: none;
            -khtml-user-select: none;
            -moz-user-select: none;
            -ms-user-select: none;
            user-select: none;
            -webkit-tap-highlight-color: rgba(0, 0, 0, 0);
        }

        .buttonMomentary {
            padding: 10px 20px;
            font-size: 24px;
            text-align: center;
            outline: none;
            color: #fff;
            background-color: #555;
            border: none;
            border-radius: 5px;
            box-shadow: 0 4px #333;
            cursor: pointer;
        }

        .buttonMomentary:active {
            background-color: #444;
            box-shadow: 0 4px #222;
            transform: translateY(2px);
        }

        .starStateTextCSS {
            font-size: 1.5rem;
            color: white;
            padding-right: 30px;
        }

        .tflStateTextCSS {
            font-size: 1.5rem;
            color: white;
            padding-left: 30px;
        }

        .dropdownbtn0 {
            padding: 10px 20px;
            font-size: 24px;
            text-align: center;
            outline: none;
            color: #fff;
            background-color: #555;
            border: none;
            border-radius: 5px;
            box-shadow: 0 4px #333;
            cursor: pointer;
        }

        .dropdownbtn0:active {
            background-color: #444;
            box-shadow: 0 4px #222;
            transform: translateY(2px);
        }

        .dropdown0 {
            position: relative;
            display: inline-block;
        }

        .dropdown-content0 {
            display: none;
            position: absolute;
            background-color: #555;
            min-width: 233px;
            overflow: auto;
            box-shadow: 0px 8px 16px 0px rgba(0, 0, 0, 0.2);
            z-index: 1;
            transform: translateY(7px);
            border: none;
            border-radius: 10px;
        }

        .dropdown-content0 a {
            color: white;
            padding: 8px 12px;
            text-decoration: none;
            text-align: center;
            display: block;
            font-size: 1.5rem;
            border: 1px solid #333;
        }

        .dropdown0 a:hover {
            background-color: #666;
        }

        .show0 {
            display: block;
        }

        .dropdownbtn1 {
            padding: 10px 20px;
            font-size: 24px;
            text-align: center;
            outline: none;
            color: #fff;
            background-color: #555;
            border: none;
            border-radius: 5px;
            box-shadow: 0 4px #333;
            cursor: pointer;
        }

        .dropdownbtn1:active {
            background-color: #444;
            box-shadow: 0 4px #222;
            transform: translateY(2px);
        }

        .dropdown1 {
            position: relative;
            display: inline-block;
        }

        .dropdown-content1 {
            display: none;
            position: absolute;
            background-color: #555;
            min-width: 233px;
            overflow: auto;
            box-shadow: 0px 8px 16px 0px rgba(0, 0, 0, 0.2);
            z-index: 1;
            transform: translateY(7px);
            border: none;
            border-radius: 10px;
        }

        .dropdown-content1 a {
            color: white;
            padding: 8px 12px;
            text-decoration: none;
            text-align: center;
            display: block;
            font-size: 1.5rem;
            border: 1px solid #333;
        }

        .dropdown1 a:hover {
            background-color: #666;
        }

        .show1 {
            display: block;
        }

        .sliderUGLWBrtnsText {
            font-size: 1.5rem;
            color: white;
        }

        .sliderUGLWSpeedText {
            font-size: 1.5rem;
            color: white;
        }

        .sliderUGLW {
            -webkit-appearance: none;
            margin: -40px;
            width: 250px;
            height: 25px;
            background: #222;
            outline: none;
            -webkit-transition: .2s;
            transition: opacity .2s;
            border-radius: 15px;
        }

        .sliderUGLW::-webkit-slider-thumb {
            -webkit-appearance: none;
            appearance: none;
            width: 35px;
            height: 35px;
            background: #555;
            border-radius: 25px;
            cursor: pointer;
        }

        .sliderUGLW::-moz-range-thumb {
            width: 35px;
            height: 35px;
            cursor: pointer;
        }

        .infoText1 {
            font-size: 1.5rem;
            color: white;
            padding-right: 25px;
        }

        .infoText2 {
            font-size: 1.5rem;
            color: white;
            padding-left: 25px;
        }

        .infoText3 {
            font-size: 1.5rem;
            color: white;
            font-weight: bold;
        }

        .authorText {
            font-size: 1.0rem;
            font-family: monospace;
            color: white;
        }
    </style>
</head>

<body>
    <!--  -->
    <!-- Heading -->
    <!--  -->
    <h2>Star Control</h2>
    <hr>


    <!--  -->
    <!-- Main Lights & Sliders -->
    <!--  -->
    <!-- Light state info text (Star & TFL) -->
    <div>
        <br>
        <span id="starStateText" class="starStateTextCSS">%STARSTATETEXT%</span>
        <span id="tflStateText" class="tflStateTextCSS">%TFLSTATETEXT%</span>
        <br><br>
    </div>
    <hr>

    <!-- Star Slider with Text -->
    <br><br>
    <div><span id="slider0ValueText" class="sliderMainText">%SLIDERTEXT0%</span>
        <br><br><br>
        <input type="range" onchange="updateSliderStar(this)" id="slider0" min="1" max="3" value="%SLIDERVALUE0%"
            step="1" class="sliderMain">
    </div>
    <br><br>

    <!-- TFL Slider with Text -->
    <div><span id="slider1ValueText" class="sliderMainText">%SLIDERTEXT1%</span>
        <br><br><br>
        <input type="range" onchange="updateSliderTFL(this)" id="slider1" min="1" max="3" value="%SLIDERVALUE1%"
            step="1" class="sliderMain">
    </div>
    <br><br><br>
    <hr>

    <!-- Strobe Button -->
    <br><button id="button0" class="buttonMomentary" onmousedown="momentaryButton(this, '1');"
        ontouchstart="momentaryButton(this, '1');" onmouseup="momentaryButton(this, '0');"
        ontouchend="momentaryButton(this, '0');">Strobe</button>
    <br><br>
    <hr>


    <!--  -->
    <!-- Underglow controls -->
    <!--  -->
    <h4>Underglow</h4>
    <br>

    <!-- Currently selected Mode - text -->
    <div><span id="uglwSelMode" class="sliderUGLWSpeedText">%UGLWSELMODETEXT%</span></div>
    <br><br>

    <!-- Mode dropdown -->
    <div class="dropdown0">
        <button onmousedown="showDropdown(dropdownID0)" class="dropdownbtn0">Mode</button>
        <div id="dropdownID0" class="dropdown-content0">
            <a onmousedown="dropdownButtonPressed(0, 0)">Static (0)</a>
            <a onmousedown="dropdownButtonPressed(0, 1)">Blink (1)</a>
            <a onmousedown="dropdownButtonPressed(0, 2)">Breath (2)</a>
            <a onmousedown="dropdownButtonPressed(0, 3)">Color Wipe (3)</a>
            <a onmousedown="dropdownButtonPressed(0, 7)">Color Wipe Random (7)</a>
            <a onmousedown="dropdownButtonPressed(0, 8)">Random Color (8)</a>
            <a onmousedown="dropdownButtonPressed(0, 9)">Single Dynamic (9)</a>
            <a onmousedown="dropdownButtonPressed(0, 10)">Multi Dynamic (10)</a>
            <a onmousedown="dropdownButtonPressed(0, 11)">Rainbow Even (11)</a>
            <a onmousedown="dropdownButtonPressed(0, 12)">Rainbow Cycle (12)</a>
            <a onmousedown="dropdownButtonPressed(0, 13)">Scan (13)</a>
            <a onmousedown="dropdownButtonPressed(0, 14)">Dual Scan (14)</a>
            <a onmousedown="dropdownButtonPressed(0, 15)">Breath Fast (15)</a>
            <a onmousedown="dropdownButtonPressed(0, 16)">Theater Chase (16)</a>
            <a onmousedown="dropdownButtonPressed(0, 17)">Theater Chase Rainbow (17)</a>
            <a onmousedown="dropdownButtonPressed(0, 18)">Running Lights (18)</a>
            <a onmousedown="dropdownButtonPressed(0, 19)">Twinkle (19)</a>
            <a onmousedown="dropdownButtonPressed(0, 20)">Twinkle Random (20)</a>
            <a onmousedown="dropdownButtonPressed(0, 21)">Twinkle Fade (21)</a>
            <a onmousedown="dropdownButtonPressed(0, 22)">Twinkle Fade Random (22)</a>
            <a onmousedown="dropdownButtonPressed(0, 23)">Sparkle (23)</a>
            <a onmousedown="dropdownButtonPressed(0, 26)">Strobe (26)</a>
            <a onmousedown="dropdownButtonPressed(0, 27)">Strobe Rainbow (27)</a>
            <a onmousedown="dropdownButtonPressed(0, 29)">Blink Rainbow (29)</a>
            <a onmousedown="dropdownButtonPressed(0, 32)">Chase Random (32)</a>
            <a onmousedown="dropdownButtonPressed(0, 33)">Chase Rainbow (33)</a>
            <a onmousedown="dropdownButtonPressed(0, 38)">Chase Blackout Rainbow (38)</a>
            <a onmousedown="dropdownButtonPressed(0, 40)">Running Color (40)</a>
            <a onmousedown="dropdownButtonPressed(0, 41)">Running Red-Blue (41)</a>
            <a onmousedown="dropdownButtonPressed(0, 42)">Running Random (42)</a>
            <a onmousedown="dropdownButtonPressed(0, 43)">Larson Scanner (43)</a>
            <a onmousedown="dropdownButtonPressed(0, 44)">Comet (44)</a>
            <a onmousedown="dropdownButtonPressed(0, 45)">Fireworks (45)</a>
            <a onmousedown="dropdownButtonPressed(0, 46)">Fireworks Random (46)</a>
            <a onmousedown="dropdownButtonPressed(0, 47)">Merry Christmas (47)</a>
            <a onmousedown="dropdownButtonPressed(0, 49)">Fire Flicker (49)</a>
            <a onmousedown="dropdownButtonPressed(0, 51)">Circus Combustus (51)</a>
            <a onmousedown="dropdownButtonPressed(0, 52)">Halloween (52)</a>
            <a onmousedown="dropdownButtonPressed(0, 55)">TwinkleFox (55)</a>
            <a onmousedown="dropdownButtonPressed(0, 56)">Rain (56)</a>
        </div>
    </div>
    <br><br><br>

    <!-- Favorite Mode - Input box with text -->
    <div><span id="slider8ValueText" class="sliderUGLWSpeedText">%SLIDERTEXT8%</span>
        <br><br>
        <form action="">
            <input type="text" onchange="updateSlider(this)" id="favoriteUGLWMode">
        </form>
    </div>
    <br><br><br>

    <!-- Color Dropdowns -->
    <div class="dropdown1">
        <button onmousedown="showDropdown(dropdownID1)" class="dropdownbtn1">Color 1</button>
        <div id="dropdownID1" class="dropdown-content1">
            <a onmousedown="dropdownButtonPressed(1, 0)">Red</a>
            <a onmousedown="dropdownButtonPressed(1, 1)">Green</a>
            <a onmousedown="dropdownButtonPressed(1, 2)">Blue</a>
            <a onmousedown="dropdownButtonPressed(1, 3)">White</a>
            <a onmousedown="dropdownButtonPressed(1, 4)">Yellow</a>
            <a onmousedown="dropdownButtonPressed(1, 5)">Cyan</a>
            <a onmousedown="dropdownButtonPressed(1, 6)">Magenta</a>
            <a onmousedown="dropdownButtonPressed(1, 7)">Purple</a>
            <a onmousedown="dropdownButtonPressed(1, 8)">Orange</a>
            <a onmousedown="dropdownButtonPressed(1, 9)">Pink</a>
            <a onmousedown="dropdownButtonPressed(1, 10)">Black</a>
        </div>
    </div>
    <div class="dropdown1">
        <button onmousedown="showDropdown(dropdownID7)" class="dropdownbtn1">Color 2</button>
        <div id="dropdownID7" class="dropdown-content1">
            <a onmousedown="dropdownButtonPressed(7, 0)">Red</a>
            <a onmousedown="dropdownButtonPressed(7, 1)">Green</a>
            <a onmousedown="dropdownButtonPressed(7, 2)">Blue</a>
            <a onmousedown="dropdownButtonPressed(7, 3)">White</a>
            <a onmousedown="dropdownButtonPressed(7, 4)">Yellow</a>
            <a onmousedown="dropdownButtonPressed(7, 5)">Cyan</a>
            <a onmousedown="dropdownButtonPressed(7, 6)">Magenta</a>
            <a onmousedown="dropdownButtonPressed(7, 7)">Purple</a>
            <a onmousedown="dropdownButtonPressed(7, 8)">Orange</a>
            <a onmousedown="dropdownButtonPressed(7, 9)">Pink</a>
            <a onmousedown="dropdownButtonPressed(7, 10)">Black</a>
        </div>
    </div>
    <br><br>
    <div class="dropdown1">
        <button onmousedown="showDropdown(dropdownID8)" class="dropdownbtn1">Color 3</button>
        <div id="dropdownID8" class="dropdown-content1">
            <a onmousedown="dropdownButtonPressed(8, 0)">Red</a>
            <a onmousedown="dropdownButtonPressed(8, 1)">Green</a>
            <a onmousedown="dropdownButtonPressed(8, 2)">Blue</a>
            <a onmousedown="dropdownButtonPressed(8, 3)">White</a>
            <a onmousedown="dropdownButtonPressed(8, 4)">Yellow</a>
            <a onmousedown="dropdownButtonPressed(8, 5)">Cyan</a>
            <a onmousedown="dropdownButtonPressed(8, 6)">Magenta</a>
            <a onmousedown="dropdownButtonPressed(8, 7)">Purple</a>
            <a onmousedown="dropdownButtonPressed(8, 8)">Orange</a>
            <a onmousedown="dropdownButtonPressed(8, 9)">Pink</a>
            <a onmousedown="dropdownButtonPressed(8, 10)">Black</a>
        </div>
    </div>
    <br><br><br><br><br>

    <!-- Brightness - Slider with text -->
    <div><span id="slider4ValueText" class="sliderUGLWBrtnsText">%SLIDERTEXT4%</span>
        <br><br><br>
        <input type="range" onchange="updateSlider(this)" id="slider4" min="0" max="255" value="%SLIDERVALUE4%" step="1"
            class="sliderUGLW">
    </div>
    <br><br><br><br>

    <!-- Speed - Slider with text -->
    <div><span id="slider5ValueText" class="sliderUGLWSpeedText">%SLIDERTEXT5%</span>
        <br><br><br>
        <input type="range" onchange="updateSlider(this)" id="slider5" min="0" max="65000" value="%SLIDERVALUE5%"
            step="1" class="sliderUGLW">
    </div>
    <br><br>

    <!-- Speed - Input box -->
    <form action="">
        <input type="text" onchange="updateSlider(this)" id="uglw_speed">
    </form>
    <br><br><br><br>

    <!-- Fadesize - Slider with text -->
    <div><span id="slider11ValueText" class="sliderUGLWSpeedText">%SLIDERTEXT11%</span>
        <br><br><br>
        <input type="range" onchange="updateSlider(this)" id="slider11" min="1" max="4" value="%SLIDERVALUE11%" step="1"
            class="sliderUGLW">
    </div>
    <br><br><br><br>

    <!-- Transition Coefficient -->
    <div><span id="slider10ValueText" class="sliderUGLWSpeedText">%SLIDERTEXT10%</span>
        <br><br>
        <form action="">
            <input type="text" onchange="updateSlider(this)" id="transcoef">
        </form>
    </div>
    <br><br>
    <hr>


    <!--  -->
    <!-- Fade Controls -->
    <!--  -->
    <!-- Fade Button -->
    %BUTTONPLACEHOLDER0%
    <br><br>
    <hr>

    <!-- Star Fade Speed - Slider with text -->
    <br><br>
    <div><span id="slider2ValueText" class="sliderFadeSpeedTextStar">%SLIDERTEXT2%</span>
        <br><br><br><br>
        <input type="range" onchange="updateSlider(this)" id="slider2" min="1" max="3" value="%SLIDERVALUE2%" step="1"
            class="sliderFadeSpeed">
    </div>
    <br><br><br>

    <!-- TFL Fade Speed - Slider with text -->
    <div><span id="slider3ValueText" class="sliderFadeSpeedTextTFL">%SLIDERTEXT3%</span>
        <br><br><br><br>
        <input type="range" onchange="updateSlider(this)" id="slider3" min="1" max="3" value="%SLIDERVALUE3%" step="1"
            class="sliderFadeSpeed">
    </div>
    <br><br>
    <hr>


    <!--  -->
    <!-- Restriction Buttons -->
    <!--  -->
    <!-- MREST Star -->
    %BUTTONPLACEHOLDER1%
    <!-- MREST Underglow -->
    %BUTTONPLACEHOLDER2%
    <!-- TFLREST Underglow -->
    %BUTTONPLACEHOLDER3%
    <br><br>
    <hr>


    <!--  -->
    <!-- Battery Management Controls -->
    <!--  -->
    <br><br>
    <!-- Current Battery Voltage text-->
    <span id="infoText3" class="infoText3">%INFOTEXT3%</span>
    <br><br><br><br>

    <!-- Battery threshold - Slider with text-->
    <div><span id="slider6ValueText" class="sliderUGLWSpeedText">%SLIDERTEXT6%</span>
        <br><br><br><br>
        <input type="range" onchange="updateSlider(this)" id="slider6" min="11.0" max="13.0" value="%SLIDERVALUE6%"
            step="0.1" class="sliderUGLW">
    </div>
    <br><br><br>

    <!-- Battery Voltage Calibration Offset - text with input box -->
    <div><span id="slider7ValueText" class="sliderUGLWSpeedText">%SLIDERTEXT7%</span>
        <br><br>
        <form action="">
            <input type="text" onchange="updateSlider(this)" id="batvoltoffset">
        </form>
    </div>
    <br><br><br>

    <!-- Battery Voltage Motor Offset - text with input box -->
    <div><span id="slider9ValueText" class="sliderUGLWSpeedText">%SLIDERTEXT9%</span>
        <br><br>
        <form action="">
            <input type="text" onchange="updateSlider(this)" id="batvoltoffsetmotor">
        </form>
    </div>
    <br><br>
    <hr>


    <!--  -->
    <!-- Footer Infos -->
    <!--  -->
    <div>
        <br>
        <!-- WiFi-RSSI - text -->
        <span id="infoText1" class="infoText1">%INFOTEXT1%</span>
        <!-- Version - text -->
        <span id="infoText2" class="infoText2">%INFOTEXT2%</span>
        <br><br>
    </div>
    <hr>

    
    <!--  -->
    <!-- Author -->
    <!--  -->
    <br>
    <span id="authorText1" class="authorText">created by Sploit | &copy; 2022 </span>

    <script>
        //*** Dropdown Functions ***
        function dropdownButtonPressed(x, y) {
            var xhr = new XMLHttpRequest();
            xhr.open("GET", "/dropdown?id=" + x + "&value=" + y, true);
            xhr.send();
        }

        function showDropdown(element) {
            var key = -1;
            if (element.id == "dropdownID0") key = 0;
            else if (element.id == "dropdownID1" || element.id == "dropdownID7" || element.id == "dropdownID8") key = 1;
            document.getElementById(element.id).classList.toggle("show" + key);
        }

        window.onclick = function (event) {
            if (!event.target.matches('.dropdownbtn0')) {
                var dropdowns = document.getElementsByClassName("dropdown-content0");
                var i;
                for (i = 0; i < dropdowns.length; i++) {
                    var openDropdown = dropdowns[i];
                    if (openDropdown.classList.contains('show0')) {
                        openDropdown.classList.remove('show0');
                    }
                }
            }
            if (!event.target.matches('.dropdownbtn1')) {
                var dropdowns = document.getElementsByClassName("dropdown-content1");
                var i;
                for (i = 0; i < dropdowns.length; i++) {
                    var openDropdown = dropdowns[i];
                    if (openDropdown.classList.contains('show1')) {
                        openDropdown.classList.remove('show1');
                    }
                }
            }
        }

        //*** Slider & Button Functions ***
        function updateButton(element) {
            var xhr = new XMLHttpRequest();
            if (element.checked) { xhr.open("GET", "/button?id=" + element.id + "&state=1", true); }
            else { xhr.open("GET", "/button?id=" + element.id + "&state=0", true); }
            xhr.send();
        }
        function updateSliderStar(element) {
            var sliderValue = document.getElementById(element.id).value;
            slider0UpdateText();
            var xhr = new XMLHttpRequest();
            xhr.open("GET", "/slider?id=" + element.id + "&value=" + sliderValue, true);
            xhr.send();
        }
        function updateSliderTFL(element) {
            var sliderValue = document.getElementById(element.id).value;
            slider1UpdateText();
            var xhr = new XMLHttpRequest();
            xhr.open("GET", "/slider?id=" + element.id + "&value=" + sliderValue, true);
            xhr.send();
        }
        function updateSlider(element) {
            var sliderValue = document.getElementById(element.id).value;
            console.log('SliderID: ' + element.id);
            console.log('SliderValue: ' + sliderValue);
            var xhr = new XMLHttpRequest();
            xhr.open("GET", "/slider?id=" + element.id + "&value=" + sliderValue, true);
            xhr.send();
        }
        function momentaryButton(element, x) {
            var xhr = new XMLHttpRequest();
            xhr.open("GET", "/momentaryButton?id=" + element.id + "&state=" + x, true);
            xhr.send();
        }

        //*** Sync Functions ***

        function syncButtons(btnID) {
            var xhttp = new XMLHttpRequest();
            xhttp.onreadystatechange = function () {
                if (this.readyState == 4 && this.status == 200) {
                    if (this.responseText == "true") {
                        document.getElementById(btnID).checked = true;
                    }
                    else {
                        document.getElementById(btnID).checked = false;
                    }
                }
            };
            xhttp.open("GET", "/btnstate?id=" + btnID, true);
            xhttp.send();
        }
        function syncSliders(sldrID) {
            var xhttp = new XMLHttpRequest();
            xhttp.onreadystatechange = function () {
                if (this.readyState == 4 && this.status == 200) {
                    document.getElementById(sldrID).value = this.responseText;
                }
            };
            xhttp.open("GET", "/sldrstate?id=" + sldrID, true);
            xhttp.send();
        }
        function slider0UpdateText() {
            switch (document.getElementById("slider0").value) {
                case "1":
                    document.getElementById("slider0ValueText").innerHTML = "Star - Automatic";
                    break;
                case "2":
                    document.getElementById("slider0ValueText").innerHTML = "Star - On";
                    break;
                case "3":
                    document.getElementById("slider0ValueText").innerHTML = "Star - Off";
                    break;
            }
        }
        function slider1UpdateText() {
            switch (document.getElementById("slider1").value) {
                case "1":
                    document.getElementById("slider1ValueText").innerHTML = "TFL - Automatic";
                    break;
                case "2":
                    document.getElementById("slider1ValueText").innerHTML = "TFL - On";
                    break;
                case "3":
                    document.getElementById("slider1ValueText").innerHTML = "TFL - Off";
                    break;
            }
        }
        function slider2UpdateText() {
            switch (document.getElementById("slider2").value) {
                case "1":
                    document.getElementById("slider2ValueText").innerHTML = "Star - Fade Slow";
                    break;
                case "2":
                    document.getElementById("slider2ValueText").innerHTML = "Star - Fade Normal";
                    break;
                case "3":
                    document.getElementById("slider2ValueText").innerHTML = "Star - Fade Fast";
                    break;
            }
        }
        function slider3UpdateText() {
            switch (document.getElementById("slider3").value) {
                case "1":
                    document.getElementById("slider3ValueText").innerHTML = "TFL - Fade Slow";
                    break;
                case "2":
                    document.getElementById("slider3ValueText").innerHTML = "TFL - Fade Normal";
                    break;
                case "3":
                    document.getElementById("slider3ValueText").innerHTML = "TFL - Fade Fast";
                    break;
            }
        }
        function slider4UpdateText() {
            document.getElementById("slider4ValueText").innerHTML = "Brightness - " + document.getElementById("slider4").value;
        }
        function slider5UpdateText() {
            document.getElementById("slider5ValueText").innerHTML = "Speed - " + document.getElementById("slider5").value;
        }
        function slider6UpdateText() {
            document.getElementById("slider6ValueText").innerHTML = "Threshold - " + document.getElementById("slider6").value + " Volt";
        }
        function slider7UpdateText() {
            var xhttp = new XMLHttpRequest();
            xhttp.onreadystatechange = function () {
                if (this.readyState == 4 && this.status == 200) {
                    document.getElementById("slider7ValueText").innerHTML = this.responseText;
                }
            };
            xhttp.open("GET", "/batVoltOffset", true);
            xhttp.send();
        }
        function slider8UpdateText() {
            var xhttp = new XMLHttpRequest();
            xhttp.onreadystatechange = function () {
                if (this.readyState == 4 && this.status == 200) {
                    document.getElementById("slider8ValueText").innerHTML = this.responseText;
                }
            };
            xhttp.open("GET", "/favoriteUGLWMode", true);
            xhttp.send();
        }
        function slider9UpdateText() {
            var xhttp = new XMLHttpRequest();
            xhttp.onreadystatechange = function () {
                if (this.readyState == 4 && this.status == 200) {
                    document.getElementById("slider9ValueText").innerHTML = this.responseText;
                }
            };
            xhttp.open("GET", "/batVoltOffsetMotor", true);
            xhttp.send();
        }
        function slider10UpdateText() {
            var xhttp = new XMLHttpRequest();
            xhttp.onreadystatechange = function () {
                if (this.readyState == 4 && this.status == 200) {
                    document.getElementById("slider10ValueText").innerHTML = this.responseText;
                }
            };
            xhttp.open("GET", "/transCoef", true);
            xhttp.send();
        }
        function slider11UpdateText() {
            switch (document.getElementById("slider11").value) {
                case "1":
                    document.getElementById("slider11ValueText").innerHTML = "Fadesize - Small";
                    break;
                case "2":
                    document.getElementById("slider11ValueText").innerHTML = "Fadesize - Medium";
                    break;
                case "3":
                    document.getElementById("slider11ValueText").innerHTML = "Fadesize - Large";
                    break;
                case "4":
                    document.getElementById("slider11ValueText").innerHTML = "Fadesize - XLarge";
                    break;
            }
        }
        function starStateUpdateText() {
            var xhttp = new XMLHttpRequest();
            xhttp.onreadystatechange = function () {
                if (this.readyState == 4 && this.status == 200) {
                    document.getElementById("starStateText").innerHTML = this.responseText;
                }
            };
            xhttp.open("GET", "/starStateText", true);
            xhttp.send();
        }
        function tflStateUpdateText() {
            var xhttp = new XMLHttpRequest();
            xhttp.onreadystatechange = function () {
                if (this.readyState == 4 && this.status == 200) {
                    document.getElementById("tflStateText").innerHTML = this.responseText;
                }
            };
            xhttp.open("GET", "/tflStateText", true);
            xhttp.send();
        }
        function infoText1UpdateText() {
            var xhttp = new XMLHttpRequest();
            xhttp.onreadystatechange = function () {
                if (this.readyState == 4 && this.status == 200) {
                    document.getElementById("infoText1").innerHTML = this.responseText;
                }
            };
            xhttp.open("GET", "/infoText1", true);
            xhttp.send();
        }
        function infoText3UpdateText() {
            var xhttp = new XMLHttpRequest();
            xhttp.onreadystatechange = function () {
                if (this.readyState == 4 && this.status == 200) {
                    document.getElementById("infoText3").innerHTML = this.responseText;
                }
            };
            xhttp.open("GET", "/infoText3", true);
            xhttp.send();
        }
        function uglwSelModeUpdateText() {
            var xhttp = new XMLHttpRequest();
            xhttp.onreadystatechange = function () {
                if (this.readyState == 4 && this.status == 200) {
                    document.getElementById("uglwSelMode").innerHTML = this.responseText;
                }
            };
            xhttp.open("GET", "/uglwSelMode", true);
            xhttp.send();
        }

        //*** Interval Functions ***

        setInterval(function () {
            syncButtons("button0");
            syncButtons("button1");
            syncButtons("button2");
            syncButtons("button3");
            syncButtons("button4");
            syncSliders("slider0");
            syncSliders("slider1");
            syncSliders("slider2");
            syncSliders("slider3");
            syncSliders("slider4");
            syncSliders("slider5");
            syncSliders("slider6");
            syncSliders("slider11");
        }, 2000);
        setInterval(function () {
            slider0UpdateText();
            slider1UpdateText();
            slider2UpdateText();
            slider3UpdateText();
            slider4UpdateText();
            slider5UpdateText();
            slider6UpdateText();
            slider7UpdateText();
            slider8UpdateText();
            slider9UpdateText();
            slider10UpdateText();
            slider11UpdateText();
            starStateUpdateText();
            tflStateUpdateText();
            uglwSelModeUpdateText();
        }, 250);
        setInterval(function () {
            infoText1UpdateText();
            infoText3UpdateText();
        }, 1000)
    </script>
</body>

</html>
    )rawliteral";