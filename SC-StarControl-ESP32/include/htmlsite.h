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
                font-size: 1.5rem;
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
                margin: 14px;
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
            }

            .sliderFadeSpeed {
                -webkit-appearance: none;
                margin: 14px;
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
                padding-right: 40px;
            }

            .sliderFadeSpeedTextTFL {
                font-size: 1.5rem;
                color: white;
                padding-left: 40px;
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
        </style>
    </head>

    <body>
        <h2>Star Control</h2>
        <hr>
        <p><span id="starStateText" class="starStateTextCSS">%STARSTATETEXT%</span><span id="tflStateText"
                class="tflStateTextCSS">%TFLSTATETEXT%</span></p>
        <hr>

        <p><span id="slider0ValueText" class="sliderMainText">%SLIDERTEXT0%</span></p>
        <p><input type="range" onchange="updateSliderStar(this)" id="slider0" min="1" max="3" value="%SLIDERVALUE0%"
                step="1" class="sliderMain"></p>
        <p><span id="slider1ValueText" class="sliderMainText">%SLIDERTEXT1%</span></p>
        <p><input type="range" onchange="updateSliderTFL(this)" id="slider1" min="1" max="3" value="%SLIDERVALUE1%"
                step="1" class="sliderMain"></p>
        <hr>

        <!-- Strobe Button -->
        <br><button id="button0" class="buttonMomentary" onmousedown="momentaryButton(this, '1');"
            ontouchstart="momentaryButton(this, '1');" onmouseup="momentaryButton(this, '0');"
            ontouchend="momentaryButton(this, '0');">Strobe</button>
        <br><br>
        <hr>

        <!-- Dropdown Menu -->
        <h4>Underglow</h4>
        <div class="dropdown0">
            <button onmousedown="showDropdown(dropdownID0)" class="dropdownbtn0">Mode</button>
            <div id="dropdownID0" class="dropdown-content0">
                <a onmousedown="dropdownButtonPressed(0, 0)">Static</a>
                <a onmousedown="dropdownButtonPressed(0, 1)">Blink</a>
                <a onmousedown="dropdownButtonPressed(0, 2)">Breath</a>
                <a onmousedown="dropdownButtonPressed(0, 33)">Rainbow</a>
            </div>
        </div>
        <div class="dropdown1">
            <button onmousedown="showDropdown(dropdownID1)" class="dropdownbtn1">Color</button>
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
            </div>
        </div>
        <br>
        <br>
        <hr>

        %BUTTONPLACEHOLDER0%
        <!-- Fade Button -->
        <hr>

        <p>
            <span id="slider2ValueText" class="sliderFadeSpeedTextStar">%SLIDERTEXT2%</span>
            <span id="slider3ValueText" class="sliderFadeSpeedTextTFL">%SLIDERTEXT3%</span>
        </p>
        <p>
            <input type="range" onchange="updateSlider(this)" id="slider2" min="1" max="3" value="%SLIDERVALUE2%"
                step="1" class="sliderFadeSpeed">
            <input type="range" onchange="updateSlider(this)" id="slider3" min="1" max="3" value="%SLIDERVALUE3%"
                step="1" class="sliderFadeSpeed">
        </p>
        <br>
        <hr>

        %BUTTONPLACEHOLDER1%
        <!-- MREST Button -->
        <hr>

        <script>
            //*** Dropdown Functions ***
            function dropdownButtonPressed(x, y)
            {
                var xhr = new XMLHttpRequest();
                xhr.open("GET", "/dropdown?id=" + x + "&value=" + y, true);
                xhr.send();
            }

            function showDropdown(element) {
            	var key = -1;
            	if (element.id == "dropdownID0") key = 0;
                else if (element.id == "dropdownID1") key = 1;
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
                        document.getElementById("slider0ValueText").innerHTML = "Star - Automatik";
                        break;
                    case "2":
                        document.getElementById("slider0ValueText").innerHTML = "Star - An";
                        break;
                    case "3":
                        document.getElementById("slider0ValueText").innerHTML = "Star - Aus";
                        break;
                }
            }
            function slider1UpdateText() {
                switch (document.getElementById("slider1").value) {
                    case "1":
                        document.getElementById("slider1ValueText").innerHTML = "TFL - Automatik";
                        break;
                    case "2":
                        document.getElementById("slider1ValueText").innerHTML = "TFL - An";
                        break;
                    case "3":
                        document.getElementById("slider1ValueText").innerHTML = "TFL - Aus";
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

            //*** Interval Functions ***

            setInterval(function () {
                syncButtons("button0");
                syncButtons("button1");
                syncButtons("button2");
                syncSliders("slider0");
                syncSliders("slider1");
                syncSliders("slider2");
                syncSliders("slider3");
            }, 2000);
            setInterval(function () {
                slider0UpdateText();
                slider1UpdateText();
                slider2UpdateText();
                slider3UpdateText();
                starStateUpdateText();
                tflStateUpdateText();
            }, 250);
        </script>
    </body>

    </html>
    )rawliteral";