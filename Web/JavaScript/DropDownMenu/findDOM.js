 
var isID = 0;
var isDHTML = 0;
var isAll = 0;
var isLayers = 0;
 
if (document.getElementById) { isID = 1; isDHTML = 1; }
else {
   if (document.all) {isAll = 1; isDHTML = 1; } 
   else {
      browserVersion = parseInt(navigator.appVersion);
   if ((navigator.appName.indexOf('Netscape') != -1) && (browserVersion == 4)) {isLayers = 1; isDHTML = 1; }
} }
 
function findDOM(objectID,withStyle) {
   if (withStyle == 1) {
      if (isID) { return (document.getElementById(objectID).style); }
      else {
         if (isAll) { return (document.all[objectID].style); }
      else {
         if (isLayers) { return (document.layers[objectID]); }
      } ; }
   }
      else {
         if (isID) { return (document.getElementById(objectID)); }
      else {
         if (isAll) { return (document.all[objectID]); }
      else {
         if (isLayers) {return (document.layers[objectID]); }
      } ; }
   }
}
 
