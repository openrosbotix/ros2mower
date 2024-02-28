TODO:
- mow action erstellen für BT und für ros2mower
    - nav2 waypoint follower action oder navigateThroughPoses
    - nav through poses, auch die orientierung muss hier erstellt werden als Teil der pose. Dazu den Winkel zum nächsten Punkt ermitteln
    - zuvor den pfad erstellen als 
- groot2 publisher
- costmap für Keepout zones erstellen in map provider

done
- checkbatteryVoltage basierend auf der Spannung
- prüfung auf battery aus ros2mower_robot entfernen. Logik wird vom BT übernommen.
- set mission server in ROBOT Klasse nur dann ändern, wenn sie sich von der aktuellen unterscheidet.
- setMission nur dann, wenn aktuelle Mission unterschiedlich ist --> gelöst über ReactiveFallback
- checkmission ergebnis ins Blackboard schreiben
- aktuelle mow area in blackboard schreiben. Darüber soll die nächste mow area bestimmt werden können
