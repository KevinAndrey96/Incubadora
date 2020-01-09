<?php
date_default_timezone_set('America/Bogota');
include_once "ConfigBD.php";

$Dato=$_POST["Dato"];
$Dato=substr($Dato, 0, -2);
//$Dato="1.68,4.23,3.65,1.96,2.54,15.69,12.2";
$Datos = explode(",", $Dato);
//$Datos[6]=substr($Datos[6], 0, -4);
if($Datos[5]==" NAN" )
	$Datos[5]=0;
if($Datos[4]==" NAN")
	$Datos[4]=0;
if($Datos[3]==" NAN")
	$Datos[3]=0;
$Q="INSERT INTO `Datos`(`T1`, `T2`, `TP`, `H1`, `H2`, `HP`, `NivelAgua`,`P1`,`P2`,`PP`) VALUES ('$Datos[0]','$Datos[1]','$Datos[2]','$Datos[3]','$Datos[4]','$Datos[5]','$Datos[6]','$Datos[7]','$Datos[8]','$Datos[9]')";
$db->query($Q);

echo "Ok";

?>