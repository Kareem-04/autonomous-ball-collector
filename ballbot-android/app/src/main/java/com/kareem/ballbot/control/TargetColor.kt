package com.kareem.ballbot.control

enum class TargetColor(val displayName: String, val acceptedLabels: Set<String>) {
    ANY(   "Any ball", setOf("red ball", "blue ball", "yellow ball", "green ball")),
    RED(   "Red",      setOf("red ball")),
    BLUE(  "Blue",     setOf("blue ball")),
    YELLOW("Yellow",   setOf("yellow ball")),
    GREEN( "Green",    setOf("green ball"));

    fun matches(label: String): Boolean =
        acceptedLabels.contains(label.trim().lowercase())
}