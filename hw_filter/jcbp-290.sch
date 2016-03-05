<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE eagle SYSTEM "eagle.dtd">
<eagle version="7.2.0">
<drawing>
<settings>
<setting alwaysvectorfont="yes"/>
<setting verticaltext="up"/>
</settings>
<grid distance="0.05" unitdist="inch" unit="inch" style="lines" multiple="1" display="yes" altdistance="0.01" altunitdist="inch" altunit="inch"/>
<layers>
<layer number="1" name="Top" color="4" fill="1" visible="no" active="no"/>
<layer number="16" name="Bottom" color="1" fill="1" visible="no" active="no"/>
<layer number="17" name="Pads" color="2" fill="1" visible="no" active="no"/>
<layer number="18" name="Vias" color="2" fill="1" visible="no" active="no"/>
<layer number="19" name="Unrouted" color="6" fill="1" visible="no" active="no"/>
<layer number="20" name="Dimension" color="15" fill="1" visible="no" active="no"/>
<layer number="21" name="tPlace" color="23" fill="1" visible="no" active="no"/>
<layer number="22" name="bPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="23" name="tOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="24" name="bOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="25" name="tNames" color="7" fill="1" visible="no" active="no"/>
<layer number="26" name="bNames" color="7" fill="1" visible="no" active="no"/>
<layer number="27" name="tValues" color="7" fill="1" visible="no" active="no"/>
<layer number="28" name="bValues" color="7" fill="1" visible="no" active="no"/>
<layer number="29" name="tStop" color="7" fill="3" visible="no" active="no"/>
<layer number="30" name="bStop" color="7" fill="6" visible="no" active="no"/>
<layer number="31" name="tCream" color="7" fill="4" visible="no" active="no"/>
<layer number="32" name="bCream" color="7" fill="5" visible="no" active="no"/>
<layer number="33" name="tFinish" color="6" fill="3" visible="no" active="no"/>
<layer number="34" name="bFinish" color="6" fill="6" visible="no" active="no"/>
<layer number="35" name="tGlue" color="7" fill="4" visible="no" active="no"/>
<layer number="36" name="bGlue" color="7" fill="5" visible="no" active="no"/>
<layer number="37" name="tTest" color="7" fill="1" visible="no" active="no"/>
<layer number="38" name="bTest" color="7" fill="1" visible="no" active="no"/>
<layer number="39" name="tKeepout" color="4" fill="11" visible="no" active="no"/>
<layer number="40" name="bKeepout" color="1" fill="11" visible="no" active="no"/>
<layer number="41" name="tRestrict" color="4" fill="10" visible="no" active="no"/>
<layer number="42" name="bRestrict" color="1" fill="10" visible="no" active="no"/>
<layer number="43" name="vRestrict" color="2" fill="10" visible="no" active="no"/>
<layer number="44" name="Drills" color="7" fill="1" visible="no" active="no"/>
<layer number="45" name="Holes" color="7" fill="1" visible="no" active="no"/>
<layer number="46" name="Milling" color="3" fill="1" visible="no" active="no"/>
<layer number="47" name="Measures" color="7" fill="1" visible="no" active="no"/>
<layer number="48" name="Document" color="7" fill="1" visible="no" active="no"/>
<layer number="49" name="Reference" color="7" fill="1" visible="no" active="no"/>
<layer number="51" name="tDocu" color="6" fill="1" visible="no" active="no"/>
<layer number="52" name="bDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="90" name="Modules" color="5" fill="1" visible="yes" active="yes"/>
<layer number="91" name="Nets" color="2" fill="1" visible="yes" active="yes"/>
<layer number="92" name="Busses" color="1" fill="1" visible="yes" active="yes"/>
<layer number="93" name="Pins" color="2" fill="1" visible="no" active="yes"/>
<layer number="94" name="Symbols" color="4" fill="1" visible="yes" active="yes"/>
<layer number="95" name="Names" color="7" fill="1" visible="yes" active="yes"/>
<layer number="96" name="Values" color="7" fill="1" visible="yes" active="yes"/>
<layer number="97" name="Info" color="7" fill="1" visible="yes" active="yes"/>
<layer number="98" name="Guide" color="6" fill="1" visible="yes" active="yes"/>
</layers>
<schematic xreflabel="%F%N/%S.%C%R" xrefpart="/%S.%C%R">
<libraries>
<library name="balun">
<packages>
<package name="SMA_EDGE_MNT">
<smd name="GND1" x="-2.54" y="0" dx="1.524" dy="5.08" layer="16"/>
<smd name="SIG" x="0" y="0" dx="1.524" dy="5.08" layer="1"/>
<smd name="GND2" x="2.54" y="0" dx="1.524" dy="5.08" layer="16"/>
<wire x1="-3.175" y1="2.54" x2="-3.175" y2="-2.2098" width="0.127" layer="51"/>
<wire x1="-3.175" y1="-2.2098" x2="3.175" y2="-2.2098" width="0.127" layer="51"/>
<wire x1="3.175" y1="-2.2098" x2="3.175" y2="2.54" width="0.127" layer="51"/>
<wire x1="3.175" y1="2.54" x2="2.54" y2="2.54" width="0.127" layer="51"/>
<wire x1="2.54" y1="2.54" x2="-2.54" y2="2.54" width="0.127" layer="51"/>
<wire x1="-2.54" y1="2.54" x2="-3.175" y2="2.54" width="0.127" layer="51"/>
<wire x1="-3.81" y1="2.54" x2="-3.81" y2="-3.81" width="0.127" layer="21"/>
<wire x1="3.81" y1="-3.81" x2="3.81" y2="2.54" width="0.127" layer="21"/>
<smd name="GND3" x="-2.54" y="0" dx="1.524" dy="5.08" layer="1"/>
<smd name="GND4" x="2.54" y="0" dx="1.524" dy="5.08" layer="1"/>
<wire x1="-2.54" y1="2.54" x2="-2.54" y2="5.08" width="0.127" layer="51"/>
<wire x1="-2.54" y1="5.08" x2="2.54" y2="5.08" width="0.127" layer="51"/>
<wire x1="2.54" y1="5.08" x2="2.54" y2="2.54" width="0.127" layer="51"/>
<text x="5.08" y="2.54" size="1.27" layer="25" rot="SR270">&gt;NAME</text>
</package>
<package name="BG291">
<wire x1="10.16" y1="11.049" x2="10.16" y2="-11.049" width="0.127" layer="21"/>
<wire x1="-10.16" y1="-11.049" x2="-10.16" y2="11.049" width="0.127" layer="21"/>
<smd name="GND1" x="-7.62" y="-10.033" dx="1.651" dy="2.54" layer="1"/>
<smd name="RF_IN" x="-5.08" y="-10.033" dx="1.651" dy="2.54" layer="1"/>
<smd name="GND3" x="-2.54" y="-10.033" dx="1.651" dy="2.54" layer="1"/>
<smd name="GND4" x="0" y="-10.033" dx="1.651" dy="2.54" layer="1"/>
<smd name="GND5" x="2.54" y="-10.033" dx="1.651" dy="2.54" layer="1"/>
<smd name="GND6" x="5.08" y="-10.033" dx="1.651" dy="2.54" layer="1"/>
<smd name="GND7" x="7.62" y="-10.033" dx="1.651" dy="2.54" layer="1"/>
<smd name="GND14" x="-7.62" y="10.033" dx="1.651" dy="2.54" layer="1"/>
<smd name="GND13" x="-5.08" y="10.033" dx="1.651" dy="2.54" layer="1"/>
<smd name="GND12" x="-2.54" y="10.033" dx="1.651" dy="2.54" layer="1"/>
<smd name="GND11" x="0" y="10.033" dx="1.651" dy="2.54" layer="1"/>
<smd name="GND10" x="2.54" y="10.033" dx="1.651" dy="2.54" layer="1"/>
<smd name="RF_OUT" x="5.08" y="10.033" dx="1.651" dy="2.54" layer="1"/>
<smd name="GND8" x="7.62" y="10.033" dx="1.651" dy="2.54" layer="1"/>
<wire x1="-10.16" y1="11.049" x2="-8.89" y2="11.049" width="0.127" layer="21"/>
<wire x1="-10.16" y1="-11.049" x2="-8.89" y2="-11.049" width="0.127" layer="21"/>
<wire x1="10.16" y1="-11.049" x2="8.89" y2="-11.049" width="0.127" layer="21"/>
<wire x1="10.16" y1="11.049" x2="8.89" y2="11.049" width="0.127" layer="21"/>
<text x="-11.43" y="-10.16" size="1.27" layer="25" rot="R90">&gt;NAME</text>
</package>
</packages>
<symbols>
<symbol name="CONSMA013.062_JACK">
<wire x1="-2.54" y1="-2.54" x2="0" y2="0" width="0.254" layer="94"/>
<wire x1="-2.54" y1="2.54" x2="0" y2="0" width="0.254" layer="94"/>
<wire x1="0" y1="0" x2="10.16" y2="0" width="0.254" layer="94"/>
<wire x1="-2.54" y1="5.08" x2="0" y2="7.62" width="0.254" layer="94"/>
<wire x1="-2.54" y1="10.16" x2="0" y2="7.62" width="0.254" layer="94"/>
<wire x1="10.16" y1="0" x2="10.16" y2="5.08" width="0.254" layer="94"/>
<wire x1="10.16" y1="5.08" x2="12.7" y2="5.08" width="0.254" layer="94"/>
<wire x1="10.16" y1="5.08" x2="7.62" y2="5.08" width="0.254" layer="94"/>
<circle x="10.16" y="7.62" radius="2.54" width="0.254" layer="94"/>
<wire x1="0" y1="7.62" x2="20.32" y2="7.62" width="0.254" layer="94"/>
<wire x1="10.16" y1="0" x2="20.32" y2="0" width="0.254" layer="94"/>
<pin name="SIG" x="25.4" y="7.62" length="middle" rot="R180"/>
<pin name="GND" x="25.4" y="0" length="middle" rot="R180"/>
<text x="0" y="-5.08" size="1.778" layer="95">&gt;NAME</text>
<text x="0" y="10.16" size="1.778" layer="96">&gt;VALUE</text>
</symbol>
<symbol name="JCBP-290+">
<wire x1="-12.7" y1="0" x2="-12.7" y2="7.62" width="0.254" layer="94"/>
<wire x1="-12.7" y1="7.62" x2="12.7" y2="7.62" width="0.254" layer="94"/>
<wire x1="12.7" y1="7.62" x2="12.7" y2="0" width="0.254" layer="94"/>
<wire x1="12.7" y1="0" x2="-12.7" y2="0" width="0.254" layer="94"/>
<pin name="RF_IN" x="-17.78" y="5.08" length="middle"/>
<pin name="GND_IN" x="-17.78" y="2.54" length="middle"/>
<pin name="RF_OUT" x="17.78" y="5.08" length="middle" rot="R180"/>
<pin name="GND_OUT" x="17.78" y="2.54" length="middle" rot="R180"/>
<text x="-12.7" y="7.62" size="1.778" layer="95">&gt;NAME</text>
<text x="-12.7" y="-2.54" size="1.778" layer="96">&gt;VALUE</text>
</symbol>
<symbol name="CONSMA013.062">
<wire x1="0" y1="0" x2="10.16" y2="0" width="0.254" layer="94"/>
<wire x1="10.16" y1="0" x2="20.32" y2="0" width="0.254" layer="94"/>
<wire x1="10.16" y1="0" x2="10.16" y2="5.08" width="0.254" layer="94"/>
<wire x1="10.16" y1="5.08" x2="7.62" y2="5.08" width="0.254" layer="94"/>
<wire x1="10.16" y1="5.08" x2="12.7" y2="5.08" width="0.254" layer="94"/>
<circle x="10.16" y="7.62" radius="2.54" width="0.254" layer="94"/>
<wire x1="0" y1="7.62" x2="20.32" y2="7.62" width="0.254" layer="94"/>
<wire x1="20.32" y1="7.62" x2="17.78" y2="10.16" width="0.254" layer="94"/>
<wire x1="20.32" y1="7.62" x2="17.78" y2="5.08" width="0.254" layer="94"/>
<wire x1="20.32" y1="0" x2="17.78" y2="2.54" width="0.254" layer="94"/>
<wire x1="20.32" y1="0" x2="17.78" y2="-2.54" width="0.254" layer="94"/>
<pin name="SIG" x="-5.08" y="7.62" length="middle"/>
<pin name="GND" x="-5.08" y="0" length="middle"/>
<text x="-5.08" y="-5.08" size="1.778" layer="95">&gt;NAME</text>
<text x="-5.08" y="10.16" size="1.778" layer="96">&gt;VALUE</text>
</symbol>
</symbols>
<devicesets>
<deviceset name="CONSMA013.062_JACK">
<gates>
<gate name="G$1" symbol="CONSMA013.062_JACK" x="-10.16" y="0"/>
</gates>
<devices>
<device name="END_MNT" package="SMA_EDGE_MNT">
<connects>
<connect gate="G$1" pin="GND" pad="GND1 GND2 GND3 GND4"/>
<connect gate="G$1" pin="SIG" pad="SIG"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="JCBP-290+">
<gates>
<gate name="G$1" symbol="JCBP-290+" x="0" y="0"/>
</gates>
<devices>
<device name="" package="BG291">
<connects>
<connect gate="G$1" pin="GND_IN" pad="GND1 GND3 GND4 GND5 GND6 GND7"/>
<connect gate="G$1" pin="GND_OUT" pad="GND8 GND10 GND11 GND12 GND13 GND14"/>
<connect gate="G$1" pin="RF_IN" pad="RF_IN"/>
<connect gate="G$1" pin="RF_OUT" pad="RF_OUT"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="CONSMA013.062">
<gates>
<gate name="G$1" symbol="CONSMA013.062" x="-10.16" y="0"/>
</gates>
<devices>
<device name="END_MNT" package="SMA_EDGE_MNT">
<connects>
<connect gate="G$1" pin="GND" pad="GND1 GND2 GND3 GND4"/>
<connect gate="G$1" pin="SIG" pad="SIG"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
</libraries>
<attributes>
</attributes>
<variantdefs>
</variantdefs>
<classes>
<class number="0" name="default" width="0" drill="0">
</class>
</classes>
<parts>
<part name="INPUT" library="balun" deviceset="CONSMA013.062_JACK" device="END_MNT"/>
<part name="FILTER" library="balun" deviceset="JCBP-290+" device=""/>
<part name="OUTPUT" library="balun" deviceset="CONSMA013.062" device="END_MNT"/>
</parts>
<sheets>
<sheet>
<plain>
</plain>
<instances>
<instance part="INPUT" gate="G$1" x="0" y="0"/>
<instance part="FILTER" gate="G$1" x="57.15" y="1.27"/>
<instance part="OUTPUT" gate="G$1" x="88.9" y="0"/>
</instances>
<busses>
</busses>
<nets>
<net name="GND_IN" class="0">
<segment>
<pinref part="INPUT" gate="G$1" pin="GND"/>
<pinref part="FILTER" gate="G$1" pin="GND_IN"/>
<wire x1="25.4" y1="0" x2="39.37" y2="0" width="0.1524" layer="91"/>
<wire x1="39.37" y1="0" x2="39.37" y2="3.81" width="0.1524" layer="91"/>
<label x="30.48" y="-5.08" size="1.778" layer="95"/>
<pinref part="FILTER" gate="G$1" pin="GND_OUT"/>
<pinref part="OUTPUT" gate="G$1" pin="GND"/>
<wire x1="74.93" y1="3.81" x2="83.82" y2="3.81" width="0.1524" layer="91"/>
<wire x1="83.82" y1="3.81" x2="83.82" y2="0" width="0.1524" layer="91"/>
<label x="76.2" y="-5.08" size="1.778" layer="95"/>
<wire x1="39.37" y1="3.81" x2="74.93" y2="3.81" width="0.1524" layer="91"/>
<junction x="39.37" y="3.81"/>
<junction x="74.93" y="3.81"/>
</segment>
</net>
<net name="RF_IN" class="0">
<segment>
<pinref part="INPUT" gate="G$1" pin="SIG"/>
<pinref part="FILTER" gate="G$1" pin="RF_IN"/>
<wire x1="25.4" y1="7.62" x2="39.37" y2="7.62" width="0.1524" layer="91"/>
<wire x1="39.37" y1="7.62" x2="39.37" y2="6.35" width="0.1524" layer="91"/>
<label x="34.29" y="15.24" size="1.778" layer="95"/>
</segment>
</net>
<net name="RF_OUT" class="0">
<segment>
<pinref part="FILTER" gate="G$1" pin="RF_OUT"/>
<pinref part="OUTPUT" gate="G$1" pin="SIG"/>
<wire x1="74.93" y1="6.35" x2="83.82" y2="6.35" width="0.1524" layer="91"/>
<wire x1="83.82" y1="6.35" x2="83.82" y2="7.62" width="0.1524" layer="91"/>
<label x="80.01" y="16.51" size="1.778" layer="95"/>
</segment>
</net>
</nets>
</sheet>
</sheets>
</schematic>
</drawing>
</eagle>
