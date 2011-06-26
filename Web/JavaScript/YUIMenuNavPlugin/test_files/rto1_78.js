
if(typeof YAHOO=="undefined"){YAHOO={};}
if(typeof YAHOO.RT=="undefined"){YAHOO.RT={};}
YAHOO.RT.qsvars={};rt_RtCk="RT";var rti_version="$Revision: 1.78 $";rti_version=rti_version.substring(11,rti_version.length-2);var rti_source;var rti_spaceid;var rti_comment;var rti_bComputeDelta=true;var rti_beaconUrl=("https:"==document.location.protocol?"https:":"http:")+"//rtb.pclick.yahoo.com/images/rt.gif";var rti_userAgent=navigator.userAgent;var rti_isNav=(navigator.appName.indexOf("Netscape")!=-1&&parseInt(navigator.appVersion)>=4&&rti_userAgent.indexOf("Opera")==-1);var rti_beaconSrc;function rt_Start(url){if(typeof url=="undefined"||!url.length||typeof url!="string"){url="";}
var start=Number(new Date());rt_setCk(rt_RtCk,"s="+start+"&u="+escape(url)+"&r="+escape(document.location),0,"/",rti_GetHostname(2));if(20<(Number(new Date()))-start){rt_rmCk(rt_RtCk);}}
function rt_Done(bSkipCheck,params,bNoBeacon){var end=Number(new Date());if("object"==typeof(bSkipCheck))
bSkipCheck=0;if("object"==typeof(params)||typeof params=="undefined")
params="";var start=0;var bSendBeacon=false;var result="";var t_done=0;var url=escape(document.location);var ref=escape(document.referrer);if(""!=rt_getCk(rt_RtCk)){start=Number(rt_getSCk(rt_RtCk,"s"));url=rt_getSCk(rt_RtCk,"u");ref=rt_getSCk(rt_RtCk,"r");rt_rmCk(rt_RtCk);t_done=end-start;bSendBeacon=true;}
for(var time_name in document.rt_times){if(typeof document.rt_times.hasOwnProperty!="undefined"&&!document.rt_times.hasOwnProperty(time_name)){continue;}
if("t_rtpage"==time_name){rt_LogTime(time_name,end-document.rt_times[time_name],true);}
else{rt_LogTime(time_name,document.rt_times[time_name]-(rti_bComputeDelta?start:0),true);}}
if(typeof document.rt_timesdelta!="undefined"&&document.rt_timesdelta["t_rtpage"]&&!document.rt_timesdelta["t_resp"]&&t_done>document.rt_timesdelta["t_rtpage"]){rt_LogTime("t_resp",t_done-document.rt_timesdelta["t_rtpage"],true);}
if(typeof document.rt_timesdelta!="undefined"){var t_other="";for(var time_name in document.rt_timesdelta){if(typeof document.rt_timesdelta.hasOwnProperty!="undefined"&&!document.rt_timesdelta.hasOwnProperty(time_name)){continue;}
t_other+=","+escape(time_name)+"|"+document.rt_timesdelta[time_name];}
if(0<t_other.length){bSendBeacon=true;params+="&t_other="+t_other.substring(1);}}
document.rt_times={};document.rt_timesdelta={};var beaconSrc="";if(bSendBeacon){var iColon;if(typeof rt_page!="undefined"&&0<rt_page.length&&(iColon=rt_page.indexOf(":"))){var spaceid=(rti_spaceid?rti_spaceid:rt_page.substring(0,iColon));var location=rt_page.substring(iColon+1);params+="&src="+location+"&spaceid="+spaceid;}
beaconSrc=rt_Beacon(url,ref,t_done,bSkipCheck,params,true);if(!bNoBeacon){rti_beaconSrc=beaconSrc;}}
rt_SetBandwidth();return beaconSrc;}
function rt_LogTime(time_name,time_value,bDelta){if(bDelta){if(typeof time_value!="undefined"){if(typeof document.rt_timesdelta=="undefined"){document.rt_timesdelta={};}
document.rt_timesdelta[time_name]=time_value;}}
else{if(typeof document.rt_times=="undefined"){document.rt_times={};}
document.rt_times[time_name]=(typeof time_value=="undefined"||0==time_value?Number(new Date()):time_value);}}
function rt_SetSource(source){rti_source=source;}
function rt_SetSpaceid(spaceid){rti_spaceid=spaceid;}
function rt_SetUrl(beaconUrl){rti_beaconUrl=beaconUrl;}
function rt_SetComment(comment){rti_comment=comment;}
function rt_AddVar(name,value){YAHOO.RT.qsvars[name]=value;}
function rt_SetComputeDelta(bDelta){rti_bComputeDelta=bDelta;}
function rt_Beacon(url,ref,t_done,bSkipCheck,params,bNoBeacon){if(typeof url=="undefined"||0==url.length){url=escape(document.location);}
if("string"==typeof(params)){var aTuples=params.split("&");for(var i=0;i<aTuples.length;i++){var aTuple=aTuples[i].split("=");if(2==aTuple.length){rt_AddVar(aTuple[0],aTuple[1]);}}}
if(rti_source){rt_AddVar("src",escape(rti_source));}
if(rti_comment){rt_AddVar("comment",escape(rti_comment));}
if(rti_version){rt_AddVar("v",rti_version);}
var u2="";if(!bSkipCheck&&url!=escape(document.location)){u2="&u2="+escape(document.location);}
var r2="";if(!bSkipCheck&&ref!=escape(document.referrer)){r2="&r2="+escape(document.referrer);}
var src=rti_beaconUrl+"?u="+url+u2+"&r="+ref+r2
+(0<t_done?"&t_done="+t_done:"");for(var name in YAHOO.RT.qsvars){if(typeof YAHOO.RT.qsvars.hasOwnProperty!="undefined"&&!YAHOO.RT.qsvars.hasOwnProperty(name)){continue;}
src+="&"+name+"="+YAHOO.RT.qsvars[name];}
if(!bNoBeacon){rti_SendBeacon();}
return src;}
function rti_SendBeacon(src){if(!src){src=rti_beaconSrc;}
if(src&&("https:"!=document.location.protocol||"https:"==src.substring(0,6))){tmpimg=new Image();tmpimg.src=src+"&ba="+rt_getSCk(rt_BaCk,"ba");}}
function rt_RemoveHandler(sType){if("load"==sType){rti_removeHandler(window,"load",rt_Done);}
else if("beforeunload"==sType){rti_removeHandler(window,"beforeunload",rt_Start);}}
rti_addHandler(window,"load",rt_Done);rti_addHandler(window,"beforeunload",rt_Start);if(typeof YAHOO.RT.ADS==='undefined'){YAHOO.RT.ADS={};}
function rt_AdBeacon()
{var protocol=("https:"===document.location.protocol?"https:":"http:");var adBeaconUrl=protocol+"//rtb.pclick.yahoo.com/images/rt_ad.gif";var location,spaceid,params="";if(typeof rt_page!=="undefined"&&0<rt_page.length&&(iColon=rt_page.indexOf(":"))){spaceid=(rti_spaceid?rti_spaceid:rt_page.substring(0,iColon));location=rt_page.substring(iColon+1);params+="src="+escape(location)+"&spaceid="+escape(spaceid);}
var adSend=false;var cnt=0;var ifrm,st_val,end_val,t_done;var iframes=document.getElementsByTagName("iframe");for(var i=0;i<iframes.length;i++){ifrm=iframes[i];st_val=YAHOO.RT.ADS[ifrm.id+"_startTime"];end_val=YAHOO.RT.ADS[ifrm.id+"_endTime"];if(typeof st_val!=="undefined"&&typeof end_val!=="undefined"){adSend=true;t_done=end_val-st_val;param1_nm="ad_u"+cnt;param2_nm="ad_done"+cnt;params+="&"+param1_nm+"="+escape(ifrm.src)+"&"+param2_nm+"="+t_done;cnt++;}}
var url,ref,sendUrl;if(adSend){url=escape(document.location.href);ref=escape(document.referrer);rt_SetUrl(adBeaconUrl);sendUrl=rt_Beacon(url,ref,0,true,params,true);rti_SendBeacon(sendUrl);rt_SetUrl(rti_beaconUrl);}}
rti_addHandler(window,"beforeunload",rt_AdBeacon);rt_BaCk="BA";var rti_startTime=0;var rti_imageSizes=[20.254,50.413,101.577,306.935,613.795,1200.623];var rti_imageNames=["y20_1.jpg","y50_1.jpg","y100_1.jpg","y300_1.jpg","y600_1.jpg","y1200_1.jpg"];var rti_imageTimes=[0,0,0,0,0,0];var rti_imageNum=0;var rti_totalTime=0;var rti_totalSize=0;function rt_SetBandwidth(){var bandwidth=rt_getSCk(rt_BaCk,"ba");var bSet=0;if("https:"==document.location.protocol){bSet=0;}
else if(""==bandwidth){bSet=1;}
else{var ip=rt_getSCk(rt_BaCk,"ip");var bTime=rt_getSCk(rt_BaCk,"t");if(""==ip||(typeof rt_ip!="undefined"&&ip!=rt_ip)){if(""==bTime||parseInt(bTime)+3600<Number(new Date())/1000){bSet=1;}}}
if(bSet){if(rt_setCk(rt_BaCk,"t="+Math.round(Number(new Date())/1000),0,"/",rti_GetHostname(2))){setTimeout("rt_TestBandwidth(true)",50);}}
else{rti_SendBeacon();}}
function rt_TestBandwidth(bInit){if(bInit){rti_imageNum=rti_totalTime=rti_totalSize=0;}
tmpImg=new Image();tmpImg.onload=rti_ImageOnload;var src="http://l.yimg.com/a/i/rt/"+rti_imageNames[rti_imageNum];rti_startTime=Number(new Date());tmpImg.src=src;}
function rti_ImageOnload(){var endTime=Number(new Date());rti_imageTimes[rti_imageNum]=(endTime-rti_startTime);rti_totalTime+=(endTime-rti_startTime);rti_totalSize+=rti_imageSizes[rti_imageNum];if(3500>rti_totalTime&&rti_imageNum<rti_imageNames.length-1){rti_imageNum++;rt_TestBandwidth();}
else{var kbps=Math.round(8*rti_totalSize*1000/rti_totalTime);var kbps2=0;if(rti_imageNum>2){var effectiveSize=0;var effectiveTime=0;for(var i=2;i<=rti_imageNum;i++){effectiveSize+=rti_imageSizes[i];effectiveTime+=rti_imageTimes[i];}
kbps2=Math.round(8*effectiveSize*1000/effectiveTime);}
if(kbps2>kbps)kbps=kbps2;rti_SetBandwidthCookie(kbps);rti_SendBeacon();}}
function rti_SetBandwidthCookie(kbps){var newCookie="ba="+kbps+
(typeof rt_ip=="undefined"?"":("&ip="+rt_ip))+"&t="+Math.round(Number(new Date())/1000);var exp=0;if(typeof rt_ip!="undefined"){var exp=new Date();exp.setTime(exp.getTime()+7*1000*60*60*24);exp=exp.toGMTString();}
rt_setCk(rt_BaCk,newCookie,exp,"/",rti_GetHostname(2));}
function rti_addHandler(elem,sType,fn,capture){capture=(capture)?true:false;if(elem.addEventListener){elem.addEventListener(sType,fn,capture);}
else if(elem.attachEvent){elem.attachEvent("on"+sType,fn);}
else{if(elem["on"+sType]){}
else{elem["on"+sType]=fn;}}}
function rti_removeHandler(elem,sType,fn,capture){capture=(capture)?true:false;if(window.removeEventListener){elem.removeEventListener(sType,fn,(capture));}
else if(window.detachEvent){elem.detachEvent("on"+sType,fn);}}
function rt_getCk(name){name=' '+name+'=';var i,cookies;cookies=' '+document.cookie+';';if((i=cookies.indexOf(name))>=0){i+=name.length;cookies=cookies.substring(i,cookies.indexOf(';',i));return cookies;}
return"";}
function rt_getSCk(name,subname){subname='&'+subname+'=';var i,subcookie;subcookie=rt_getCk(name);subcookie='&'+subcookie+'&';if((i=subcookie.indexOf(subname))>=0){subcookie=subcookie.substring(i+subname.length,subcookie.indexOf('&',i+subname.length));return subcookie;}
return"";}
function rt_setCk(name,value,exp,path,domain,sec){var nameval=name+"="+value;var str=nameval+
((exp)?"; expires="+exp:"")+
((path)?"; path="+path:"")+
((domain)?"; domain="+domain:"")+
((sec)?"; secure":"");if((name.length>0)&&(nameval.length<4000)){document.cookie=str;return(value==rt_getCk(name));}
return 0;}
function rt_rmCk(name){var exp=new Date(90,1,1);return rt_setCk(name,"",exp.toGMTString(),"/",rti_GetHostname(2));}
function rti_GetHostname(level){var hostname=document.location.hostname;if("number"==typeof(level)&&0<level){var aParts=hostname.split(".");aParts.reverse();hostname="";for(var i=0;i<level&&i<aParts.length;i++){hostname="."+aParts[i]+hostname;}
hostname=hostname.substring(1);}
return hostname;}