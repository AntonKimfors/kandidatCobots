# kandidatCobots
Cobots 2019 - ROS

Logg:
Eftersom jag inte körde via en router satte jag statisk IP på datorn, 192.168.1.125, 255.255.255.0.

Startar nod mot roboten eller nått
roslaunch ur_modern_driver ur10_bringup.launch robot_ip:=192.168.1.116

Startar planning-nånting?
roslaunch ur10_moveit_config ur10_moveit_planning_execution.launch

Startar Rviz
roslaunch ur10_moveit_config moveit_rviz.launch config:=true

Publicerar hur lederna sitter ihop (tror jag)
roslaunch ur_description ur10_upload.launch


Andra gången jag körde gick det mycket bättre. Kopplade bara upp mig mot roboten, körde kommandona i ordningen som står ovan och så funkade allt, även i rviz. Gick bra att flytta på roboten via Rviz. KOM IHÅG ATT SÄNKA HASTIGHETEN OCH TRYCKA PLAN FÖRST.

Skrivit en fil, objects.py, som är tänkt att lägga till scene objects till rviz, men inte fått det att funka.


Fått in objecten ia Rviz nu. Men den kan ändå planera genom dem?

Ändrat i move_robot till att köra mellan 2 positioner, dock inte i en loop.


ROS2:
	Får inte ett paket att bygga som gör att några andra paket inte kan byggas heller. Paketet är qt_gui_cpp, det som blir fel är i “sip”, SIPNULLPTR.

colcon build --symlink-install --packages-skip ros1_bridge qt_gui_cpp qt_gui_core rqt_gui_cpp rqt


Försöker skapa ett nytt package i ros2_ws/src/ros2. Gör packaget med ros2 pkg create “name”, raderar cmake-filen och lägger in en setup.py. Ändrar om i package.xml och lägger till setup.cfg. Setup.py, setup.cfg och package.xml kollar jag från demo_nodes_py och gör samma. Bygger med colcon build, men kan inte köra med “ros2 run commander demo”. Om jag kör “ros2 pkg executables” kommer “commander demo” upp. Men det funkar ändå inte.

Lagt till “resource”-mappen, så nu är setup.py helt identiska(innan var data_files bortkommenterade). Var de inte bortkommenterade kunde man inte builda commander alls. Nu funkar det att builda, men får fortfarande samma fel när jag försöker köra.

Kopierade istället filer i demo_nodes_py och la in i dess setup.py, funkar.

Slutade med att jag raderade det jag gjort med “ros2 pkg create” och istället kopierade hela demo_nodes_py och bara döpte om saker. FUNKAR!
Kan vara så att problemet är att python-filerna inte får vara i samma mapp/nivå som setup.py.

För att importera egna meddelanden ska jag göra ett package bara för meddelandena, så gör Endre. Kopierade unique_identifier_msgs och ändrade om namn i package.xml och cmakelist. När man ändrar i ett message behöver man ta bort mappen under /build och /install och builda paketet igen.

Lyckats göra en nod som publicerar på /cmd1 med Command och subscribar på /state1 med State. Den pubbar som den ska, kan läsa av i annan terminal, och om jag pubbar från en annan terminal med State subbar noden rätt. Men när jag startade bridgen länkades inte /cmd1 över till ROS1. Dock så hade jag igång allt innan jag startade bridgen, kanske var fel.

Gjorde en pub och en sub i ros1 ros_intro_course som subbar/pubbar på /cmd1, fick de att funka. Lade till mapping_rules.yaml-filer enligt Endre’s github-guide. Går att bridgea /cmd1 från ros2 till ros1! La också till så att det går att skapa en static bridge för just topicen /cmd1, man kan göra det för flera andra topics. Även här följde jag Endre’s guide. Gjorde samma sak för /state1, testat att köra både två static bridges samt en dynamic bridge, båda funkar.

Skriva om informationsprotokoll?? Typ ha med vilken sorts produkt så commander kan ta olika “spår”.

Började skriva på command-noden, börjar med att få igenom en produkt genom en station. Gjorde lite mer generell lösning, även om det kanske borde gjorts i en senare iteration. Men skrev först if-satsar för varje station för sig innan jag gjorde while-sats som kollade alla. Skapade arrayer som håller 3 meddelanden var.

Arrayen funkade inte som jag tänkt, det var bättre att skapa meddelanden direkt i den och editera dom. Hittade också rclpy.spin_once() som jag tror jag kommer behöva använda mycket.

Fått igenom en produkt genom en station. Nästa steg är antingen att köra flera produkter genom en station eller en produkt genom flera stationer. Finns förmodligen en del som kan abstraheras.

Får igenom flera produkter genom en station utan problem. Får igenom flera produker genom flera stationer! Ska en station sätta init=true innan stationen framför har kvitterat nästa command? Behövs product_in_station? Skrivit om alla meddelanden till engelska och lagt in fler kommentarer. Abstraherat så mycket jag kan, vet dock inte hur jag ska göra med subscribersen och deras callback-metoder, de är hårdkodade just nu.

Löste så att subscribersen är generellt skapade, woho! Gjorde om flera while-loopar till for-loopar. Planerar att ta bort product_in_station samt fixa så att den yttersta loopen inte är hårdkodad.

Bytte ut init, executing och finished till en enda variabel, state, som bara ska ha ett av de 3 värdena. Klurat ut hur jag kan får bort hårdkodningen i yttersta loopen på ett rimligt sätt, implementerar imorn.

Tog bort så mycket hårdkodning som går. Fick bort product_in_station helt och hållet. Gjorde 2 nya metoder för att abstrahera mera. Följande behövs för att testa i terminalen:
ros2 topic pub /state0 commander_msgs/State "{state: init, cmd: '', message: ''}"

    • Visa vad jag gjort/tänkt för de andra grupperna(och min egna)
    • Fixa vpn mot noderna, se Endre’s post i slack.
