# 2025-08-13: The Project is Public!

## Milestone
Today, I've made the repository public and started this development log. This marks the official start of the public phase of the project.

## Current Status
The majority of the conceptual CAD work for the core 3-axis mechanics is complete (around the 90% mark). I've been so focused on design iterations and preparing for the first printing tests that I haven't committed the CAD files to the repository yet.

## Next Steps
My immediate goal is to clean up the models and push the complete set of F3D/STEP files to the /cad directory by this Saturday (August 16th).

## Comment
If someone rly see this **hey,have a nice day! :>** i hope what i make that to this saturday cuz with this i need to start making PCB its little scary but we need this to 50 kHz. <Cya> 

## 2025-08-24: Stage II Begins - Electronics & First Working Prototype!

### Milestone
This weekend marks the official start of the electronics development phase. The core mechanical and electronic architectures are now finalized. Most importantly, the first PLA prototype of the 10:1 reducer module was printed, assembled, and **it works flawlessly!**

### Current Status
Following the successful test of the 10:1 reducer, I will be uploading its final STL files to the /manufacturingdirectory. The PDF overview of the electronics architecture is now also available in/hardware/docs.

The initial assembly of the reducer revealed a critical kinematic conflict: the drive did not work. After analysis, I identified a mistake in the clearance calculation for the output flange fingers. The holes in the cycloidal discs were recalculated and enlarged, which completely solved the issue.

### Next Steps
The primary focus now shifts to designing the **Power Blade** electronics. In parallel, I will continue printing and testing the complete mechanical assembly. I'm considering creating a **Telegram channel** to post real-time development updates, including videos and failures. Let me know if you'd be interested!

### Comment & Key Discovery
I have to admit, when the first reducer assembly seized up, I was worried about the entire design. But after thinking through the kinematics, I realized a fundamental principle for balanced cycloidal drives with a rigid output cage.

The standard formula for hole clearance, Hole_Diameter = Pin_Diameter + (2 * Eccentricity), is only valid for a single-disc or independently suspended dual-disc setup.

For a **rigidly connected, balanced (counter-rotating) dual-disc setup**, the required clearance is doubled. The correct formula is:

**Hole_Diameter = Pin_Diameter + (4 * Eccentricity)**

This was a scary realization, as it required making the holes in my very compact reducer significantly larger. However, the design had enough material to accommodate this. After reprinting the discs with the corrected hole diameter, the reducer works perfectly. This is a crucial finding for anyone building this type of drive.

This week, I will try to develop the full Power Blade schematic. And again i write this at midnight... <Cya guys, sleep well>

## 2025-09-02: Little sick but with power blade architecture!

### Milestone
Power blade architecture is DONE! The short shoulder module is printed.

### Current Status
The architecture was a little easy for me because I had been thinking about it for two weeks, so I just sat down and made it. But with printing, I am continuing my war with the 3D printer. It just stopped in the middle of the night while I slept and ruined about 90% of the print. I need to start again and buy more plastic, but the printer is far from me, so I will restart printing next weekend ":/"

### Next Steps
The next step is to totally finish the blade's schematic and start to test it in SPICE programs. In parallel, I need to pick components for the blade. And of course, printing, printing, printing....

### Comment & Key Discovery
All week I was sick, and after work, I just lay in bed and thought about how much I needed to do but couldn't. BUT on the weekend, I was fine and finished the architecture. That was a victory! -^-

## 2025-09-07: POWER BLADE SCHEME IS 90% DONE!!

### Milestone
Power blade scheme is 90% done. Printing new shoulder module.

### Current Status
Updated settings for 3D printer and start printing part of shoulder module again. Ears printed and i think it okay. Today i buyed shafts and other steel staff they gonna come like... 2-3 weeks... I added datasheets and file "components" for (You won't believe it :> ) components for boards and add all components for axis channel. My win today is scheme power blade done on 90% its just need to make footprints and past. Ye i needed to test power blade today in spice program but without footprints its unreal and i dont make it not proud for that..

### Next Steps
The next step is stay same test and finishing scheme and after i can make pcb.

### Comment & Key Discovery
**DAY NOT OVER WHILE I DONT LAY!** 
Weekend again was hard cuz i have no time for help familly and no time for this robotic arm, but i sit here and now is totaly early morning i writing this and go to sleep. Printer is printing night going to over. on this week i will be study and work and free time on week is die. Have a good night guys <cya>

## 2025-09-14: Real-World Assembly & Design Evolution

### Milestone
The first full two-stage reducer prototype was assembled and tested. While encountering critical issues, the tests successfully validated the core kinematic design and led to a significantly improved and more robust architecture.

### Current Status & Key Discoveries
This weekend was dedicated to the first physical assembly of the two-stage balanced cycloidal reducer. The test was a partial success:

    The Good: The core mechanism works. When assembled, all parts fit, and the kinematics are correct — the drive rotates and provides reduction. The new, longer stator pins, when press-fitted (with a hammer), provide excellent structural rigidity without interfering with the cycloidal discs' motion.

    The Bad (Problems Found):

        Structural Failure: The bottom wall of the main reducer shaft proved to be too thin. It cracked and crumbled under the stress of assembly (e.g., inserting an M8 nut).

        Stator Pin Instability: The stator pins were not sufficiently constrained and could be dislodged, leading to mechanism failure.

    The Ugly (Resource Loss): Lost two major prints (~800g of PETG and ~40 hours of print time) this past week due to a corrupted SD card file and suboptimal print settings that caused delamination. This is a painful but valuable lesson in process reliability.

### Design Evolution (The Solutions)
Based on the test results, I've developed a new, more robust design:

    Motor-to-Reducer Coupling: The initial concept for the input shaft was flawed. The final design will be: a pneumatic collet fitting (with metric M6 thread) screwed into a heat-set brass insert in the 3D-printed reducer shaft. The entire assembly will be locked by a transverse pin ("the nail") secured with a retaining ring. This provides a robust, balanced, and modular interface.

    Stator Assembly: The stator pin instability will be solved by:

        Closing the open holes in the stator plates.

        Adding an intermediate stator plate between the two stages for added support.

        Increasing the contact surface area between the stators and the pins.

The CAD models will be updated with these improvements next week. The only potential delay is sourcing the non-standard M6 heat-set inserts.

### Next Steps & Plan Adjustment
The initial plan to start the PCB layout this week is postponed. The unexpected but critical mechanical redesign takes priority.

    This Week (Evenings): Continue with the Power Blade schematic design. The goal is to have it 100% complete by the next weekend.

    Next Weekend: Finalize the mechanical redesign in CAD and begin printing the new, improved parts.

    Documentation: I will update the repo with the final printer settings, add component libraries, etc.

### Comment
This was the first major "slip" in the schedule due to the demanding university timetable and unforeseen design flaws. I had planned to be working on the schematic this weekend but spent almost all of it on mechanical assembly and problem-solving. This is frustrating but also incredibly productive. The design is now an order of magnitude better.

In same situations i remember this quote:

    You know, the world isn't run by the laws written on paper. It's run by people. Some according to laws, others not. It depends on each individual how his world will be, what he will make of it... I dreamed of fortunes, cars, freedom, women, respect... I got it all more or less, but with it came prison, constant fear, and the blood of my comrades.

    — Mafia: The City of Lost Heaven

But unlike that story, **we don't give up so easily.** There is more to come and rest will be after revolution in robots :)

Thanks for following along. <Cya guys!>

## 2025-09-15: little steps 

### Milestone
I finished schematics for components everything except `ADC` and add `PRINTER.md` to docs 

### Comment

**I think for little commits on week i will be use milestones and comments, cuz i cant doing many work for my robot and u can see other information like next steps u can see upper.**

For today i maked 2 scheme of components DC/DC and LDO 5V and i bought components for connect reducer shaft and motor shaft. after they come i assembly reducers and test it and i think now that will be immortal connection ~~(sounds like joke)~~ but anyway we can see it later. For other i add my config for 3d printer to `docs/guides/PRINTER.md` u can check what settings i using for printing mb its can help u :)

## 2025-09-16: schematics for components are completed !!!

### Milestone
I completed scheme for STM32G4 and add all schematics to repository !!!!!

### Comment
So so so that was too long for one component but all functions for every pin from datasheet in schem and that now have footprint. 
I change MC cuz i little mistake with price for `STM32G474CEU6` u can read about that to `hardware/docs/components.md`. time to sleep.. <Cya>

# 2025-09-23: Principle scheme is complete!

## Milestone
Completed the schematic; simulations and printing are underway

## Current Status
I completed the schematic, started printing the second part of the shoulder module, and successfully assembled the shoulder module on the shaft. I also started the simulation using the INA240 in LTspice

## Next Steps
Finish simulations and start PCB design and create a Telegram channel

## Comment
This weekend was tough: I started around 9:00 PM and worked until about 4:30 AM on Saturday and 5:00 AM on Sunday. On Monday I worked and fixed the schematic — the pin headers for the joke turned out to be harder than 60% of the circuit design. I spent 4 hours figuring out how to use PCI — it's just a connector I can use for the board. Before that I considered a mezzanine connector but couldn't find one compatible with my board that didn't cost almost as much as the whole board with all components. Now we're using PCI at a reasonable price. During the week I will create a Telegram channel to document activity like videos and fails, since using Git for that is inconvenient; I'll be there all week.

# 2025-09-23: Simulation INA240 are complete!!!!

### Milestone
* Add all spice schematics for axis channel blade
* Create directory `simulatons` 
* Made 4 simulations for `INA240` amplifier.
* Made telegram channel 

### Comment
All week i working and afer work i just lay and its all what i did and that will be true if i was a just good worker who woudnt sleep tomorrow, for that after work maked simulations and learn LTspice...khm.. i working in this program before but with simple simulations and now i tryed be serios so today we have complete simulation for first component! <Good night~>

# 2025-09-28: ALL SIMULATIONS COMPLETE. STARTING PCB!!!!

## Milestone
Completed all simulations and the final schematic for the Axis Channel. Remodeled the head of the shoulder module and the shaft for the 10:1 reducer.

## Current Status
- **Simulations:** Successfully completed and verified the DC/DC, Current Amplifier, and Half-Bridge circuits.
- **Documentation:** Created a guide on how to import SPICE models.
- **CAD:** Remodeled the shoulder modules and the reducer shaft. Started printing the main shoulder part.

## Next Steps
- **Start the PCB layout** for the Axis Channel blade.
- **Finish** the CAD models for the reducer and shoulder assembly.

## Comment

I thought the simulations would be the easy part - just load models and run. The DC/DC simulation, on the other hand, was done in 30 minutes. The half-bridge, however, took an entire day just to get it to work at all.

In the end, I got the basic simulation running and saw the square waves, which is a success. Honestly, I hoped to simulate a real "hell" scenario for it with all the parasitics and protections, but that's a project in itself, and I just don't have that much time to spend only on simulations.

So I made a pragmatic engineering decision: I based the final schematic on proven reference designs from datasheets and application notes, using LTspice just for a basic "does it switch?" sanity check, without the deep, "hard tests". It's a calculated risk, but a necessary one to move forward.

Now, I continue my war with the reducer models. It needs so many iterations of printing parts, which is not easy when I can only print 2 days a week. I spent most of this weekend just modeling because I messed up the sizes for the main shaft bearing... I don't know how it happened, but it is what it is.

So, time is again not on my side. Looks like another night of 4 hours of sleep... Thanks for your attention, have good dreams :)

# 2025-10-06: SCHEMATICS ARE DONE. PCB HELL BEGINS.

## Milestone
* Started printing the lower part of the long shoulder module.
* Configuration of Altium for starting the PCB is over.
* Created all remaining components for the Axis Channel blade with their footprints.
* Imported all components into the PCB project.

## Current Status
- **Schematics:** Replaced all default Altium components with my own custom ones.
- **Libraries:** Built a complete library with every single component for the Axis Channel blade.
- **PCB:** The PCB file is created. The board outline is set to 100x100mm. The layer stack is defined. All components have been imported.

## Next Steps
- **PCB:** FINALLY START IT!!!!!
- **CAD:** Continue printing and assembling parts.
- **LTspice:** Run one or two more simulations for the LDOs.

## Comment

It's 3:40 AM on a Monday morning.... **[dying sounds]**

That was insane, really. So many hours creating footprints, hunting for datasheets, and fighting with Altium settings. But it's finally over. We are starting a new part of this strange story!

**AND for now!!!** <Have a good night, guys :)>

## 2025-10-13: PCB Hell might not BEGIN, but it could start this week...

### Milestone
* Finished printing the long lower part of the shoulder module.
* <I use Arch btw>...
* Completed simulations for the 12V, 8V, and 5V DC/DC converters.
* Corrected the DC/DC schematics in Altium and validated them in LTspice.

### Current Status
- **Printing:** The lower part of the shoulder module is done. The new support settings are working well.
- **LTspice:** Finished the DC/DC simulations and found a major problem in my original schematic. I'm not sure how it even worked before, but it's correct now. In short, I had issues with the feedback and COMP circuitry. It's fixed now.
- **Altium:** Corrected the Axis Channels chematic. I will start the PCB layout this week.
- **Arch:** Installed Arch Linux on my new laptop. I'll be using it for all STM programming and other development.

### Next Steps
- **PCB:** FINALLY START THE LAYOUT! ~~(This time for sure...)~~
- **CAD:** Print the upper part of the shoulder module.
- **Arch:** Finish configuring the development environment.

### Comment
This weekend was just tr... khm...

I forgot my laptop charger in another city, so the entire weekend was lost, and I literally did nothing. However, on Saturday, I received my new laptop ~~(a ThinkPad, running Arch btw)~~ which was the only good news.

I spent all of Saturday evening and all of Sunday installing Arch(~~on a ThinkPad, btw~~) and just finished it today.

Now I have the proper tool for thepost-PCBsteps. Today, I managed to fix the schematics and finalize the DC/DC simulations.

That was just some bad luck, and I hope it's the last bad weekend for a while ~~(nope)~~.

Thanks for your attention, <have a good night!> :)

# 2025-10-20: WE ARE BACK! 55% OF THE DRIVER IS DONE.

## Milestone
* Started printing the long upper part of the shoulder module.
* Completed the placement of all main power components (half-bridges, amplifiers, gate drivers).

## Current Status
- **Printing:** I've started printing the upper part of the shoulder module after making fixes to the CAD model.
- **Altium:** The Axis Channel schematic is fully corrected. This week, I'm diving deep into the PCB layout.

## Next Steps
- **PCB:** Finish component placement and begin the routing phase.
- **CAD:** Test assemble the long shoulder module and buy the 10mm shafts.

## Comment

This was just a very good weekend, and I can't comment on that any other way.

Everything I wanted to do, I did!

I spent all night on the placement of the MOSFETs, and along with them, I placed the amplifiers and gate drivers. And that's just... finally, it's done!!

`Just start the task.`
        `Just finish the task.`
                `That's how I will build this robot!`

<Have a good night, guys :)>

# 2025-10-21: Little Refactor

## Milestone
* Refactored the entire placement of the main power stage components. The primary goal was to drastically shorten the path from the gate drivers to the MOSFET gates.

## Comment
Before this refactoring, the trace length to the upper MOSFET's gate was around `~10mm`. After a complete re-layout of the power core, the new path is now only **`1.5mm - 1.8mm`**. This is a massive improvement that will significantly reduce gate loop inductance, leading to faster, cleaner switching and lower thermal losses.

# 2025-10-27: Placing is Over

## Milestone
* Finished placing the RS-485 receivers, the MCU-ADC, and the 5V DC/DC converter.
* Started printing the long upper part of the shoulder module... again...
* Tested new hole sizes for the pins in the 10:1 reducer.
* Dialed in new settings for Orca Slicer.

## Current Status
- **PCB:** I've finished placing 90% of the components. The last 10% are the two remaining DC/DC converters.
- **Printing:** This weekend's main theme. I broke the upper part of the shoulder module while trying to fit a bearing. After that, I changed my settings and printed a new stator, shaft, and outer flange. Then I started printing the upper part of the long shoulder module again. And, to top it off, I broke the lower part of the short module during assembly. It was a bad print from the start and was just defective all this time.

## Next Steps
- **PCB:** Finish component placement and begin the routing phase. ~~(Still same)~~
- **CAD:** Test assemble the long shoulder module and buy the 10mm shafts.

## Comment

Just an insane weekend. I finally finished the majority of the placement, and I think the routing will be easy because all the components are placed very close together, allowing me to use copper pours for many connections.

I broke TWO parts this weekend... AAAA... *khm*. But now I know that one of them was trash from the beginning, and the second one *can* fit the bearing, but only if you don't use a hammer...

A very good weekend. I hope the same for you :)

<Have a good night, sleep well guys>

# 2025-11-03: Little Necessary Work

## Milestone
*   Finished component placement for the Axis Channel blade.
*   Finished printing the long upper part of the shoulder module.

## Comment
Just some necessary work is done. Tomorrow, the plan is to finish the reducer assembly, start the PCB routing, and begin printing the lower part of the short shoulder module.

And now... <have a good night~>

# 2025-11-09: FINALLY, LAYOUT!

## Milestone
*   Replaced the MCU and its supporting components on the layout.
*   Completed the initial routing for the entire Top Layer of the Axis Channel blade (~50% of total routing).

## Current Status
- **PCB:** Literally what I said above, haha.
- **Printing:** The new head for the long shoulder module broke because the layers didn't merge properly. When I tried to press-fit a bearing (gently, really!), it just went *CRRRRR* and... *khm*. Anyway, I'll be reprinting this part (this will be the third try).

## Next Steps
- **PCB:** Finish routing!
- **CAD:** Print the head of the long shoulder module *again*, and finally `buy the 10mm shafts`.

## Comment

Phew... it's 4 AM again, and I'm sitting here, trying to make little steps to create this thing. I was a little sad when the shoulder module part just broke again, but then I tried pressing a bearing into the second hole, and it didn't break! It even held up while I was pressing it in, which gives me hope :).

I've also made a decision: first, I'll finish the routing, and *second*, I'll place the PCIe connector on the schematic. I need to understand where the routes can connect to the PCIe connector most efficiently to create the shortest paths, and only then will I finalize the connections on the main schematic.

After the part broke, I just took a hammer and smashed it to see how good the infill was. And honestly, it wasn't bad. Of course, it could be better, but it's a really solid plastic part. When I hit it, it just deformed, and only after about 10 hits did it shatter into small pieces. Yeah, some walls looked like they weren't solid, but maybe that's just the limit of 3D printing, because when I hit it, it reacted like a solid part.

So, yeah... I think it's time to sleep. <Have cool dreams~>

# 2025-11-10: Updated settings for Orca

## Current Status
- **Orca slicer** updated orca slicer settings

## Comment
U know what ?? `updated orca slicer settings!!` ~~(insane right?)~~

# 2025-11-16: Little Steps in Layout

## Milestone
*   Successfully assembled the head-shoulder module.
*   Learned all about vias. 
*   Made the layout for the GND and PWR planes.
*   Finally bought the 10mm shafts.

## Current Status
- **PCB:** I spent the entire day trying to figure out how to properly place vias for the GND and PWR planes, but now I finally know how to do it. And now, I need to go to sleep :(
- **Printing:** Finally pressed the bearings in without breaking the part! Now starting to print the base.

## Next Steps
- **PCB:** Finish routing the bottom side!
- **CAD:** Print the base.

## Comment
This weekend felt very short. I had to visit a lesson at the university this week, which is in another city, and by the time I got back, a day was gone.

So I only had one real day, and I spent all of it just understanding how to place vias correctly. For now, I need to go to sleep, but at least I know how to do it. I think the actual work of placing them will be pretty simple and fast now.

<have a good night guys ~>

# 2025-11-18: THE BEAST HAS A NERVOUS SYSTEM

## Milestone
* All signal paths are now routed.
* The "quiet" side of the board is essentially complete.

## Comment
I like how it's going. I think we can do it.

<Have a good night, guys. We're almost there.>

# 2025-11-24: LAYOUT - OVER.

## Milestone
*   **PCB:** Finished the PCB Layout / Routing.
*   **Mechanics:** Tested the new connection mechanism between reducer stages (Success).
*   **Printing:** Started printing the stator for the rotary support.

## Current Status
-   **PCB:** Yeah, we did it. The next step is a final error check (DRC), ordering the boards, and then... the scary part: Programming.
-   **Printing:** The base wasn't printed because the extruder got stuck/clogged. But I did some thinking... why spend 5kg of plastic on a base that will still be lightweight? I decided to use a **cheap, heavy concrete block** instead.
    *   Started printing the rotary support (had to cut angles slightly to fit the print bed).
    *   Spent 4 hours fixing the jammed extruder. Learned a lesson: after cleaning plastic and reassembling, you need to give the printer time to "chill," or the thermal sensor won't work right.
-   **CAD:** Designed a new mechanism to connect the two reducer stages. I made vertical slots for steel pins in the shaft, slots in the outer flange, and an 8mm plastic pin in the center for alignment. It's perfectly centered now.

## Next Steps
-   **PCB:** Order the boards!
-   **CAD:** Print the rotor for the rotary support.

## Comment
Okay, I can only sleep 4 hours again, but I finished the layout and wrote this log.

The next stage — Programming — is coming, and it's a little scary because the other stuff isn't fully finished. Drivers need to be ordered, mechanics aren't fully printed... but we need to go down this road. We can't stay still — that is the main rule.

The order itself is a challenge. Waiting 1 month... for what? To potentially see fire from my board and then wait 1 month again? It's a risk, but I knew these nuances from the start. I understand them.

If it really works... I can't imagine how happy I will be.
And next month — exams. That's also a challenge.
So, conditions are not friendly. But when were they ever? :)

We broke through it.
<Have a good night, sleep well guys>

# 2025-12-19: ALIVE!.. Maybe Alive...

## Milestone
*   Gerber, BOM, and CPL files for the Axis Channel blade are generated and ready for production.
*   The 3D printer's hotend has been replaced; printing has resumed (after a month of waiting).
*   Fixed the main problem with the reducer.
*   `components.md` has been refactored into a more structured format.

## Current Status
-   **PCB:** Waiting for the "go" signal from my contact to place the board order. The logistics are complicated, so this is currently on hold.
-   **Firmware:** The lab bench is set up. I have a dev board for motor control, the STM32H7A3 dev board, a Tang Nano FPGA board, three BLDC motors, optical and magnetic encoders, a lab PSU, and a ton of wires. The goal for this month is to get the core firmware up and running.
-   **Printing:** The new hotend is installed. Tried to print the massive slew drive. The part is warping and lifting, but I will fix it. I think it's bad settings or G-code, but I'll probably need to build an enclosure for the printer anyway...
-   **Documentation:** I completely rebuilt `components.md`. The old "just throw it in" style was great for brainstorming but terrible for actual work and comparison. The new version is structured and much more usable.

## Next Steps
-   **Firmware:** Start writing and debugging the core firmware on the dev boards.
-   **PCB:** Push for the board order.
-   **CAD:** Successfully print the slew drive and the reducer parts.

## Comment
It feels like a whole month just vanished. The last commit was on November 24th...

Between work getting intense (late nights, releases) and university exams hitting hard, the robot had to take a backseat. I spent days that should have been for studying on generating production files, and now... waiting for logistics.

It's frustrating. While the boards are in limbo, I need to finish my exams and, on my free days, start the firmware. I have everything I need to start writing and testing the code. By the time the real hardware arrives, the firmware should be 80% ready.

Just sorry for this month... ~~it will happen again.~~

<Have a good night, guys. The fight continues.>

# 2025-12-24: General Kenobi!

## Milestone
*   Updated components.md.
*   Successfully printed the slew drive.

## Comment
You know how I'll name the next dev log... *khm*.

This month was dominated by exams, and I just couldn't work on the robot full-time. But now, with the last exam of the year on Dec 26th and the next one not until Jan 11th, I'll have plenty of time to develop the firmware.

The guy who's ordering my boards says he's ready to try and place the order in the coming days.

Printing is going well, too! I made a cardboard enclosure for my printer, and the slew drive was successfully printed with new, faster settings.

It was a hard month, but now I believe it's over, or at least it's giving me a little break. Anyway, we are reaching the middle point of the project!

Thanks for your attention :)

<Good night, guys :))>


# 2026-01-03: Hello there!

## Milestone
*   **FPGA Core:** Successfully completed ~80% of the FPGA code.
*   **CAD:** Started printing the final version of the reducer.~~(and fixed the printer)~~

## Current Status
-   **Firmware:** Written the modular architecture in SystemVerilog.
    - `system_storage`: The CSR brain of the axis.
    - `pwm_generator`: Center-aligned PWM for FOC.
    - `deadtime`: Hardware protection for MOSFETs.
    - `quadrature_decoder`: Glitch-filtered encoder reading.
    
    *Key Discovery:* Spent some time fighting with the encoder signal. Turns out the Omron encoder with Open Collector output needs strong external pull-up resistors to 3.3V. Once added, the FPGA saw the signal instantly.

-   **CAD:** 
    -   **Upgrade:** Installed a new PEI build plate.
    -   **Repair:** Fixed critical mechanical play in the printer. It turns out the toolhead was misaligned due to an assembly error. I tightened all the V-wheels, and now everything is working fine.

## Next Steps
-   **Firmware:** Connect the PWM output to the real gate driver and finalize the FPGA core.
-   **PCB:** Push for the board order.
-   **CAD:** Finish printing reducer parts and start printing the rotor for the slew drive.

## Comment
It's 2:00 AM.

Sleep is.. what is this.. khm..

FPGA is very interesting, very strong, very fast AND!! very scary!!

Honestly, I just can't understand it sometimes... **Assembler feels easier!!!**

But after two days of trying to decipher these hieroglyphs, I finally started to understand, and to be honest — it's simple. On an STM32, you have a thousand functions with strange names and you need to know them all. But on an FPGA, you just have logic gates that haven't changed in the last 100 years. It feels like playing chess for the first time.

As for CAD, I'm so glad for the new PEI plate. The adhesion is insane — I actually need to heat the plate just to remove PETG parts. 

I also fixed the backlash today. Spent 3 hours on it because I had to disassemble the entire printer, reassemble it, realize I missed something, disassemble it again... aaaaand finally assembled it right.

<Have aaaa very good night guys :) Happy holidays!>

# 2026-01-06: FPGA CORE COMPLETE!

## Milestone
*   **FPGA Core:** Successfully completed the FPGA core architecture.
*   **CAD:** Printed and assembled the final version of the reducer!!!
*   **PCB:** Boards are ordered.

## Current Status
-   **Firmware:** All FPGA modules are written and verified. The FPGA is ready to connect to the MCU and drivers to spin the motor! But... this also means I now need to write the code for the MCU.
-   **CAD:** I assembled the reducer and it works!! It's a little tight (I will adjust tolerances in the next iteration), but... **GUYS, IT'S A 100:1 RATIO, IT FITS IN THE PALM OF MY HAND, AND IT WORKS!**
-   **PCB:** Boards are finally ordered! In about two weeks, we'll see the Moment of Truth (and hopefully no magic smoke).

## Next Steps
-   **Firmware:** Develop the MCU firmware and connect the PWM output to the real gate driver.
-   **CAD:** Refine reducer tolerances, start printing the rotary support, and update the CAD models.

## Comment

You know what time it is? 2:00 AM! Again!

But I'm just happy because I'm done with the FPGA part and can finally move to the next step of making this robot alive. ^^

<Have a goooood night!>

# 2026-02-08: ADC 400ns without CPU!

## Milestone
*   **FPGA Core:** Updated code to test the new ADC setup and adjusted delays.
*   **CAD:** Successfully printed the rotary support and performed a test assembly.
*   **MCU:** Implemented a bare-metal, zero-CPU-overhead Dual ADC-to-SPI Data Pipeline.

## Current Status
-   **Firmware:** The FPGA core is ready and waiting for the other parts of the system. I've tested the new ADC setup with it, and it works! Latency is projected to be around **400 ns**.
-   **CAD:** I fought with the printer all month for just two parts, but the rotary support is finally finished. I think I've learned all its quirks. (If you really try to build this, I hope you buy a better printer, but my goal is to prove you can make my robot on ANY printer).
-   **PCB:** Still waiting ;-; .

## Next Steps
-   **Firmware:** Make the motor spin using the full chain: ADC -> FPGA -> STM32H7. Maybe even start on FOC.
-   **CAD:** Update the model for the Shoulder_Clevis_Mount and figure out how to print this monster. I think it will eat more than 1kg of PETG.

## Comment

You know what? I can't believe a whole month just passed. I spent almost all of it trying to print, but this printer just didn't want to cooperate. I spent a huge part of my time just trying to understand what I was doing wrong. Two recommendations: 1. Wash your build plate (really, it's insane how well it works). 2. Don't buy an End... *khm*. Change your Z-offset while the printer is printing the first layer, and if it doesn't stick, make the gap smaller.

And the whole month I was fighting with the ADC. I tried to configure it so that when the ADC sends data to SPI, a leg of the MCU would catch it and raise a logic level. But the first week ended with zero results. The second week, I actually made it work, but the gap between the data and the logic signal was **20 µs**. I can't find the words to describe how sad it was to see that. After a whole weekend, I got it down to **6 µs**. A good result in comparison, but VERY, VERY bad for my goal. I need the data after 1-2 µs max; 0.5 µs would be perfect, leaving me 9.5 or 19.5 µs for the other things in the FOC cycle.

I don't know what I was doing in the third week, maybe I just deleted all my SPI and ADC code. But last week, I just sat on it after work and study, like 6 hours a day until 2 AM. And this Friday morning, I finished it. I can see that my ADC is working! It's just unbelievable. And now I've polished the code and even made a Makefile (ha-ha) for easy use later.

Sorry for the long pause, guys!

<HAVEEEEE AAAA GOOOOOOOOOOOOOOOD night :> >

# 2026-02-12: Architecture Document 

### Milestone
*   **docs:** Created `architecture.md` — full system architecture document.
*   **CAD:** Strarted printing shoulder clevis mount (cabin) and it`s eat 970 gr PETG with 80% fill (im happy).

### Comment
2:33 AM. Classic.

many bla bla bla but it is necessary

<Have a good night ~>

# 2026-03-12: The Beginning of the End

## Milestone
*   **FPGA Core:** Finished `top.sv` for FOC, encoder, and ADC. Full system integration complete.
*   **MCU:** Finished FOC implementation and integrated it into the complete system.
*   **CAD:** Printed the last main structural part.
*   **PCB:** Boards are VERY NEAR.

## Current Status
-   **Firmware:** All parts are working together as one system. **My architecture is correct.** On dupont wires, I achieved 20 kHz FOC. The full real-time chain works end-to-end: FPGA reads encoders and ADC currents from G4, packs them into a 256-bit SPI frame, sends to the H7, the H7 runs FOC and sends PWM commands back — all at 20 kHz, on a pile of dupont wires.
-   **CAD:** The shoulder clevis mount is printed. We now have all the parts to start assembling the body of this plastic hero.
-   **PCB:** Boards are coming to my house around March 20th. ~~hope they will work~~

## Next Steps
-   **Firmware:** Freeze while assembling the body.
-   **CAD:** Start assembling all parts of the robot and polish the fit of each piece.

## Comment

We have come to the end of this work, and we will end this at any cost.

I can't get used to the fact that every new step with electronics and code is a month of work. And I really do work on it — sitting for hours, two or more nights a week until 4 or 5 AM, trying, trying, trying again and again. And the funniest part? Not understanding what isn't working. Is it the code? A dupont wire? Did I assemble the circuit wrong? Maybe the encoder is on the wrong side? That uncertainty is the hardest part of all of this.

**But anyway, like everything else, it just needs time. And what can stop me now? Huh?**

**The firmware part is over!**

Let's just remember what I wanted to finish from the start: CAD, PCB, and... firmware. Now we just need to make it all work together. We have all the parts for it.

**Time to finish this.**

**AND NOOOWW..** time to go to sleep.

<Have a good night, good person! :> >

# **2026-04-24: Yep, it's me again. And I'm not dead.**

## Milestone
*   **Docs:** Fixed the `architecture.md` file.
*   **CAD:** Repaired the printer and printed the 60-tooth rollers; designed a motor holder; assembled the shoulder modules; printed and adjusted the actuator module; fixed the reducer models; developed a new 20:1 reducer and ordered materials for it; and finally, assembled and tested the rotary support.
*   **PCB:** Tested all board systems (they work!) and updated the Gerber files.

## Current Status
*   **CAD:** It's been a sea of troubles, but I managed to get everything I listed above done.
*   **PCB:** THE BOARDS WORK! ...well, with one tiny issue. I accidentally shorted a trace under a suppressor in the third half-bridge. A quick fix later, and everything is running smoothly. I've tested the DC/DC converters, spun a motor, and successfully programmed the microcontroller. The Gerbers are updated with the fix, so if you're brave enough, you can order them now.
*   **Docs:** I accidentally deleted the old architecture file. When I finally noticed, I just fixed it. No further comments.

## Next Steps
*   **CAD:** Keep assembling all the parts.
*   **Docs:** Write my thesis.

## Comment
This past month hasn't been the most exciting, I'll admit. There were no major milestone breakthroughs—just a whole lot of grinding. I've been assembling, printing, writing my thesis, and fixing one small issue after another. Sorry for the radio silence on GitHub, but I honestly didn't know what to write. It's not that things are bad, just that it's been a ton of little tasks—all hands-on work, spending money on bearings, shafts, and other hardware.

One interesting discovery: trimmer shafts. They're incredibly cheap, strong enough for the roller bearings, and conveniently 8mm in diameter.

A huge amount of time went into printing. I think the next dev log will be after I'm done with my thesis. If you want to see what I'm up to in the meantime, you can follow my Telegram channel.

Thanks for your attention. **The work doesn't stop.**

<Have a good night>

# 2026-03-12: First axis assembled (almost)

## Milestone
*   **Assembling:** First axis assembly is almost complete.
*   **Base:** Finished the epoxy granite base and uploaded the casting guide.

## Current Status
-   **Assembling:** 2 reducers and the rotary support are fully assembled. Currently trying to screw on the clevis mount.
-   **Base:** hmm... what did I do... forgot..

## Next Steps
-   **Assembling:** Assemble the second axis.
-   **CAD:** Continue assembling all the parts of the robot and polish the fit of each piece.

## Comment

I made the base out of epoxy granite, and it felt like doing stonemason work all day. My room looks like a dump again. I have a ton of parts laying around, I need to assemble them, buy more embedded screws, aaaaand!!! that's it. 

It took a lot of time to build the two reducers, but thankfully they work on the first try! Maybe not perfectly yet, but they work. 

Just a few last steps and the robot will come alive in this very, very positive world... I think its first words will be "why me".

First commit that's not at 4 AM... huh.. <Have a good day guys!>


# 2026-07-05: That was a long way

## Milestone
*   **Assembling:** Second axis assembled and tested.
*   **CAD:** Developed encoder holders for the first 3 real axes.

## Current Status
-   **Assembling:** Reducers for the 2nd and 3rd axes are assembled. The 2nd axis was successfully tested with a 10kg load (or 25Nm) and is now fully assembled.
-   **CAD:** Finished the design of the 3 encoder holders for axes 1, 2, and 3.

## Next Steps
-   **Assembling:** Assemble the final axis.
-   **CAD / CNC:** Start developing the 500W CNC module.
-   **Firmware:** Start developing the main control loop and inverse kinematics.

## Comment

That was an interesting experience. Really. This past year was hard. The milestone idea for this project was born last May, and looking back at it now, I just think: "Is a whole year really gone?.." And the robot still isn't finished. It's a little sad, and my ego is slightly hurt, but let's look at the bigger picture.

What did I have when this story started (and it's not over in this moment :) )? 

Last year, all I knew was that the LM358 is a good chip... and that's it. You want to ask about mechanics? Good question... nothing. I knew nothing about mechanics. Maybe code? Again, no. Just 4 Arduino lessons, and honestly, I didn't understand a single word of the code when I started.

I just said: "I'm going to build a robot, and to hell with the difficulties, I'll make it anyway!" 

Why did I say that? <I don't know if you're asking this, but I'll answer anyway -^-> 
Because everyone tells you "it's just a diploma," "it's just university," "maybe you can do it, maybe you can't, it doesn't matter." But I saw how others are building this world, and they aren't building the world I want to live in. Maybe those are big words for me, but it's true. Along the way, I just focused on giving it everything I could. Maybe it won't work in the future, maybe I'll stop the project, but this is my answer to everyone who thought I couldn't do what I wanted. 

Why this monologue? To say this: If you think you can't do something, remember me. Tell yourself: "If this guy could do all of this from scratch, I can do whatever I believe in." 

If you don't want to do something, that's normal. I wouldn't do anything either, but I have a paranoid head and I just go out of my mind if I stop working :>

Now, I am a Lead Embedded Engineer (it's my third day!), and this month I won't be able to physically assemble the robot, but I plan to start on the main loop and inverse kinematics and upload them to Git.

Sorry for the 2 months of silence. I had to chill and write my diploma. I already defended it, and next week I'm just going to pick it up! Then I can finally say I'm done with university. And now I work in the capital of my country.

That was an incredible experience. I just hope the world doesn't end in the next year so I can finish my robot and you can watch this development! 

<Have a hell of a good day!>

# 2026-07-12: free weekend

## Milestone
*   **Firmware:** Wrote the main part of the upper control loop.

## Current Status
-   **Firmware:** I now have the main part of the upper loop: inverse kinematics, the servo code, the trajectory planner, and hardware sin/cos on the FPU. It's written and the math checks out, but it isn't compiled or wired into the running cascade yet.

## Next Steps
-   **Firmware:** Test it and finish the whole upper control loop.
-   **Assembling:** Assemble the final axis.
-   **CAD / CNC:** Start developing the 500W CNC module.

## Comment

Keep developing, keep working!

and <Havee a goood niiight :> >

# 2026-08-02: Work, hard work

## Milestone
*   **CAD / CNC:** Selected an 800W spindle and created a CAD model for the adapter.

## Current Status
-   **CAD / CNC:** Upgraded the spindle choice for the CNC module to 800W (up from 500W). Found a blueprint for the spindle holder and designed the CAD model for the adapter.

## Next Steps
-   **Firmware:** Test it and finish the entire upper control loop.
-   **Assembling:** Assemble the final axis.
-   **CNC Hardware:** Choose an MCU to control the spindle and communicate with the robot. Select a motor driver for it.
-   **CNC Firmware:** Order the remaining parts for the CNC system and write the control/communication code to interface with the robot.

## Comment

Because of work, I can't go home to grab my robot, and I have to work all week. I really hope this first adjustment phase and the first month of my new job wrap up soon so I can finally get back to thinking about my project. If not, I hope things settle down soon anyway. I like my new job, but the robot and the CNC module remain my top goals. Thanks to everyone who is still following along! :)

And what? Have a... Good... What? <Night!>

# 2026-08-04: Work, hard work *upd!*

## Milestone
*   **CNC Hardware:** Selected the CNC kit and an ESP32-S3 for the CNC module.

## Current Status
-   **CAD / CNC:** I selected a kit from AliExpress with a pump, VFD, spindle, and spindle holder (around $100), as well as the MCU. The MCU is an ESP32-S3-N16R8, and I chose a TTL-to-RS485 module for communication.

## Next Steps
-   **Firmware:** Test it and finish the entire upper control loop.
-   **Assembling:** Assemble the final axis.
~~- CNC Hardware: Choose an MCU to control the spindle and communicate with the robot. Select a motor driver for it.~~
-   **CNC Firmware:** Order the remaining parts for the CNC system and write the control/communication code to interface with the robot.

## Comment

Trying out a quick style update for the dev log, as I didn't want to create a whole new file with a long title just for a short text.

<Have a good night> I'll order everything once I get paid!... though it'll take about a month to get here... yee....

# 2026-10-08: S*t happens

## Milestone
*   **CNC:** Add architecture of cnc for esp.

## Current Status
-   **CAD / CNC:** All on my board and i add all files for start make it alive.

## Next Steps
-   **Firmware:** Test it and finish the entire upper control loop.
-   **Assembling:** Assemble the final axis.
-   **CNC Firmware:** ~~Order the remaining parts for the CNC system and~~ write the control/communication code to interface with the robot.

## Comment

No comments so many GOAT live events with GOAT trash events on every week, mb i start be bad for this... but i dont stop disgrace yourself!

<gooood night have guys!> 
