<!--
var menuTop = 45;
var menuLeft = 400;
 
var domSMenu = null;
var oldDomSMenu = null;
var t = 0;
var lDelay = 20;
var lCount = 0;
var pause = 100;
 
function popMenu(menuNum) {
   if (isDHTML) {
      t = 2;
      if (oldDomSMenu) {
         oldDomSMenu.visibility = 'hidden';
         oldDomSMenu.zIndex = '0';
         t = 2;
         lCount = 0;
      }
      var idMenu = 'menuHead';
      var domMenu = findDOM(idMenu,0);
      var idMenuOpt = 'menuHead' + menuNum;
      var domMenuOpt = findDOM(idMenuOpt,0);
 
      var idSMenu = 'menu' + menuNum;
      var domSMenu = findDOM(idSMenu,1);
   if (isID || isAll) {
         var menuLeft = (domMenu.offsetLeft) + (domMenuOpt.offsetLeft) + 3;
         var menuTop = (domMenu.offsetTop) + (domMenu.offsetHeight) + 3;
         }
      if (isLayers) {
         var menuLeft = document.layers[idMenu].layers[idMenuOpt].pageX - 3;
         var menuTop = domMenu.pageY + domMenu.clip.height - 3;
      }
      if (oldDomSMenu != domSMenu) {
         domSMenu.left = menuLeft +'px';
         domSMenu.top = menuTop +'px';
         domSMenu.visibility = 'visible';
         domSMenu.zIndex = '100';
         oldDomSMenu = domSMenu;
      }
      else { oldDomSMenu = null; }
   }
   else { return null; }
}
 
function delayHide() {
   if ((oldDomSMenu) && (t == 0)) {
         oldDomSMenu.visibility = 'hidden';
         oldDomSMenu.zIndex = '0';
         oldDomSMenu = null;
         lCount = 0;
         return false;
   }
   if (t == 2) { lCount = 0; return false; }
   if (t == 1) {
         lCount = lCount + 1;
         if (lDelay <= lCount) { t = 0; }
         if (lDelay >= lCount) { setTimeout('delayHide(' + t + ')',pause); }
   }
}
//-->
 
