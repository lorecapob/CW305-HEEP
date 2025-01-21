- [ ] Instruction register composed of 4 8 bit registers byte addressable. Actually, the size of the instruction register
      can be defined in the reg block. Each byte can be addressed by the byte count parameter (see interface timings)
- [x] CU in a separate file
- [ ] Instruction valid bit set as 1 when the whole istruction has been written 
- [ ] Use the #TODO expression when something needs to be completed
- [ ] The new top wrap top level contains both the bridge and xheep
- [ ] The whole wrap simulation requires the exact USB timing show in the whitepaper

- [ ] Modify the bridge in a way it can set the exit loop flag.
- [ ] The bridge have to reset the instruction valid flag?
- [ ] Add just the core-v-mini-mcu module or everything else there is inside tb_system?
- [ ] Add another register for the new address
