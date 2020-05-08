#pragma once

#include <string>

static const std::string SVG_HTML_FILE_NAME = "circle_ga.html";
static const std::string OPEN_GROUP = "<g>";
static const std::string CLOSE_GROUP = R"(</g>
)";
static const std::string ID_BEGIN = R"(<g id="turn)";
static const std::string ID_END = R"(" style = "display:none;fill:green;">
)";
static const std::string CIRCLE_BEGIN = R"(<circle cx=")";
static const std::string CIRCLE_MIDDLE = R"(" cy=")";
static const std::string CIRCLE_END = R"(" r="6"/>
)";

//<circle cx = "270" cy = "720" r = "6" style = "fill:red;stroke-width:1" / >

static const std::string FILE_START = R"(
<? xml version = "1.0" encoding = "UTF-8"?>
<svg xmlns = "http://www.w3.org/2000/svg" version = "1.1" width = "1800" height = "950" fill = "rgb(206, 224, 230) " >

<text id = "populationText" x = "10" y = "40" style = "font-family:sans-serif;font-size:40px;fill:black">Population Id</text>

	<rect x = "100" y = "100" width = "1658" height = "808" style = "fill:none;stroke:black;stroke-width:8" />
	<circle cx = "929" cy = "504" r = "202" style = "fill:none;stroke:black;stroke-width:4" />
	<circle cx = "929" cy = "504" r = "6" style = "fill:red;stroke-width:1" />


)";


static const std::string FILE_END = R"(
</svg>

<script>

var turn = 0;
var turnStr = "turn";
function showNextSim() {
	if (turn > 0) {
		var previousTurnElementId = turnStr + (turn - 1);
		document.getElementById(previousTurnElementId).style.display = "none";
	}

	var currentTurnElementId = turnStr + turn;
	document.getElementById(currentTurnElementId).style.display = "block";
	changePopulationText(turn);
	++turn;
}

function showPreviousSim() {
	changePopulationText("Id");

	--turn;
	var currentTurnElementId = turnStr + turn;
	document.getElementById(currentTurnElementId).style.display = "none";

	if (turn > 0) {
		var previousTurnElementId = turnStr + (turn - 1);
		document.getElementById(previousTurnElementId).style.display = "block";
		changePopulationText(turn - 1);
	}
}

function wholeSimulation(groupIdx) {
	setTimeout(function() {
		var currentTurnElementId = turnStr + groupIdx;
		var element = document.getElementById(currentTurnElementId);

		if (element === null) {
			return;
		}

		if (groupIdx > 0) {
			var previousTurnElementId = turnStr + (groupIdx - 1);
			document.getElementById(previousTurnElementId).style.display = "none";
		}

		element.style.display = "block";
		changePopulationText(groupIdx);

		wholeSimulation(++groupIdx);
	}, 200);
}

function clicked(evt) {
	var e = evt.target;
	var dim = e.getBoundingClientRect();
	var x = evt.clientX - dim.left;
	var y = evt.clientY - dim.top;
	y = 3000 - y;
	alert("x: " + x + " y:" + y);
}

function changePopulationText(populationNumber) {
	document.getElementById("populationText").textContent = "Population " + populationNumber;
}

window.onkeydown = function(e) {
	var key = e.keyCode ? e.keyCode : e.which;

	// Numpad "+"
	if (107 == key) {
		showNextSim();
	}
	// Numpad "-"
	else if (109 == key) {
		showPreviousSim();
	}
	// Numpad "*"
	else if (106 == key) {
		wholeSimulation(turn);
	}
}

</script>
)";
