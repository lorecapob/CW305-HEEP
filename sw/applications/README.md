# Description of available applications

- ***aes_c***: extended version of the [`tinyAES128`](https://github.com/kokke/tiny-AES-c) C implementation. It supports AES EBC, CTR and CBC modes with several S-Boxes, more or less resistant to power side-channel attacks. The `main` program perform a simple encryption and decryption on a fixed plaintext using a fixed key and checks the correctness of the generated ciphertext.

- ***aes_c_only_encrypt***: it uses the same `AES` implementation of the `aes_c` application, but the `main` program executes a series of encryption only on a variable plaintext. Each encryption is triggered by a rising edge on the input `GPIO PIN 3`. Also, a rising edge on the output `GPIO PIN 4` is triggered when an encryption is started. This application can be used to collect power traces for side-channel attack purposes.

- ***aes_c_masked_only_encrypt***: same application as the previous one, but in this case a [`masked AES`](https://github.com/CENSUS/masked-aes-c) implementation is used.

- ***ascon_c***: Work in progrss

- ***ascon_asm_rv32i***: Work in progrss

- ***cw305_heep_hello***: simple `Hello World` program executed by the synthesized `X-HEEP` microcontroller on the `CW305` board. Text messages are automatically translated to `UART` transactions.
See FPGA constraint file `cw305.xdc` for the UART pin mapping.

- ***cw305_blink_uart***: simple test program that flashes the built-in blue LED, connected to `GPIO PIN 2`, and sends the LED status via UART.

- ***cw305_external_trigger***: simple program which checks the trigger functionality used for the other applications.