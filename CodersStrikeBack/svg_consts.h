#pragma once

#include <string>

static const std::string ENEMY = "ENEMY";
static const std::string MY = "MY";

static const std::string SVG_HTML_FILE_NAME = "coders_strike_back.html";
static const std::string SVG_HTML_FILE_NAME_ENEMY_BEGIN = "coders_strike_back_enemy_";
static const std::string SVG_HTML_FILE_NAME_MY_BEGIN = "coders_strike_back_my_";
static const std::string SVG_HTML_FILE_NAME_END = ".html";
static const std::string COMMA_SEPARATOR = ",";
static const std::string SPACE_SEPARATOR = " ";
static const std::string OPEN_GROUP = "<g>";
static const std::string CLOSE_GROUP = R"(</g>
)";
static const std::string TURN_TEXT_BEGIN = R"(<text id = "turnText" x = "10" y = "256" style = "font-family:sans-serif;font-size:256px;fill:black">Turn )";
static const std::string TURN_TEXT_MIDDLE = R"( : )";
static const std::string TURN_TEXT_END = R"( Team Simulation</text>
)";
static const std::string ID_BEGIN = R"(<g id="turn)";
static const std::string ID_END = R"(" style = "display:none;fill:green;">
)";
static const std::string CIRCLE_BEGIN = R"(<circle cx=")";
static const std::string CIRCLE_MIDDLE = R"(" cy=")";
static const std::string CIRCLE_END = R"(" r="600" stroke="black" stroke-width="16" fill="rgb(204, 51, 0) "/>
)";
static const std::string TEXT_BEGIN = R"(<text x = ")";
static const std::string TEXT_MIDDLE = R"(" y = ")";
static const std::string TEXT_STYLE = R"(" style = "font-family:sans-serif;font-size:256px;fill:black">)";
static const std::string TEXT_END = R"(</text>
)";
static const std::string POLYLINE_BEGIN = R"(<polyline points = ")";
static const std::string POLYLINE_MY_TEAM_END = R"( " style = "fill:none;stroke:rgb(0,255,0);stroke-width:10" />
)";
static const std::string POLYLINE_ENEMY_TEAM_END = R"( " style = "fill:none;stroke:rgb(0,0,255);stroke-width:10" />
)";

static const std::string FILE_START = R"(
<? xml version = "1.0" encoding = "UTF-8"?>
<svg xmlns = "http://www.w3.org/2000/svg" version = "1.1" width = "16000" height = "9000" fill = "rgb(206, 224, 230) " style = "background-color:brown" >

<text id = "populationText" x = "10" y = "512" style = "font-family:sans-serif;font-size:256px;fill:black">Population Id</text>
)";

static const std::string FILE_END = R"(
</svg>

<script>

var endSimReached = 0;
var turn = 0;
var turnStr = "turn";
function showNextSim() {
	if (endSimReached) {
		loadNextTurn();
	} else {
		var currentTurnElementId = turnStr + turn;
		var elem = document.getElementById(currentTurnElementId)
		
		if (typeof(elem) != 'undefined' && elem != null) {
			elem.style.display = "block";
			changePopulationText(turn);
		}
		else {
			endSimReached = true;
			return;
		}

		if (turn > 0) {
			var previousTurnElementId = turnStr + (turn - 1);
			document.getElementById(previousTurnElementId).style.display = "none";
		}

		++turn;
	}
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
	// Page up
	else if (33 == key) {
		loadNextTurn();
	}
	// Page down
	else if (34 == key) {
		loadPreviousTurn();
	}
}

function loadNextTurn() {
	var turnText = document.getElementById("turnText").textContent
	var turnTextSplitted = turnText.split(" ");
	var turnIdx = parseInt(turnTextSplitted[1]);
	var currentTeam = turnTextSplitted[3];
	
	var newFile = "coders_strike_back_";
	if ("ENEMY" == currentTeam) {
		newFile += "my_";
		newFile += String(turnIdx) + ".html";
	}
	else {
		newFile += "enemy_";
		newFile += String(turnIdx + 1) + ".html";
	}
	
	window.location.assign(newFile);
}

function loadPreviousTurn() {
	var turnText = document.getElementById("turnText").textContent
	var turnTextSplitted = turnText.split(" ");
	var turnIdx = parseInt(turnTextSplitted[1]);
	var currentTeam = turnTextSplitted[3];

	var newFile = "coders_strike_back_";
	if ("ENEMY" == currentTeam) {
		newFile += "my_";
		newFile += String(turnIdx - 1) + ".html";
		--turnIdx;
	}
	else {
		newFile += "enemy_";
		newFile += String(turnIdx) + ".html";
	}

	if (turnIdx >= 0) {
		window.location.assign(newFile);
	}
}

</script>
)";
