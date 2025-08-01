/**
 * \file
 * \brief  Example Configuration
 *
 * \copyright (c) 2017 Microchip Technology Inc. and its subsidiaries.
 *            You may use this software and any derivatives exclusively with
 *            Microchip products.
 *
 * \page License
 *
 * (c) 2017 Microchip Technology Inc. and its subsidiaries. You may use this
 * software and any derivatives exclusively with Microchip products.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIPS TOTAL LIABILITY ON ALL CLAIMS IN
 * ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
 * TERMS.
 */

#ifndef CLOUD_CONFIG_GCP

#include "cloud_wifi_ecc_process.h"
#include <stdio.h>
#include "definitions.h"
#include <stdint.h>
#include "cryptoauthlib.h"
#include "ecc_types.h"
#include "atcacert/atcacert_client.h"
#include "atcacert/atcacert_pem.h"
#include "m2m_types.h"
#include "nm_common.h"
#include "nm_debug.h"
#ifdef CLOUD_CONNECT_WITH_MCHP_CERTS
#include "tng/tng_atcacert_client.h"
#include "tng/tng_atca.h"
#include "tng/tngtls_cert_def_1_signer.h"
#else
#include "../../../cust_def_signer.h"
#include "../../../cust_def_device.h"
#endif

#include <wolfssl/internal.h>

uint8_t subject_key_id[20];

#define MAX_TLS_CERT_LENGTH         1024
#define SIGNER_PUBLIC_KEY_MAX_LEN   64
#define SIGNER_CERT_MAX_LEN         600 //Set Maximum of existing certs
#define DEVICE_CERT_MAX_LEN         600
#define CERT_SN_MAX_LEN             32
#define TLS_SRV_ECDSA_CHAIN_FILE    "ECDSA.lst"
#define INIT_CERT_BUFFER_LEN        (MAX_TLS_CERT_LENGTH*sizeof(uint32_t) - TLS_FILE_NAME_MAX*2 - SIGNER_CERT_MAX_LEN - DEVICE_CERT_MAX_LEN)

static const char* bin2hex(const void* data, size_t data_size)
{
    static char buf[256];
    static char hex[] = "0123456789abcdef";
    const uint8_t* data8 = data;

    if (data_size * 2 > sizeof(buf) - 1)
    {
        return "[buf too small]";
    }

    for (size_t i = 0; i < data_size; i++)
    {
        buf[i * 2 + 0] = hex[(*data8) >> 4];
        buf[i * 2 + 1] = hex[(*data8) & 0xF];
        data8++;
    }
    buf[data_size * 2] = 0;

    return buf;
}

static int8_t certs_append_file_buf(uint32_t* buffer32, uint32_t buffer_size,
                                    const char* file_name, uint32_t file_size,
                                    const uint8_t* file_data)
{
    tstrTlsSrvSecHdr* header = (tstrTlsSrvSecHdr*)buffer32;
    tstrTlsSrvSecFileEntry* file_entry = NULL;
    uint16_t str_size = (uint8_t)strlen((char*)file_name) + 1;
    uint16_t count = 0;
    uint8_t *pBuffer = (uint8_t*)buffer32;

    while ((*pBuffer) == 0xFF)
    {

        if (count == INIT_CERT_BUFFER_LEN)
        {
            break;
        }
        count++;
        pBuffer++;
    }

    if (count == INIT_CERT_BUFFER_LEN)
    {
        // The WINC will need to add the reference start pattern to the header
        header->u32nEntries = 0;                    // No certs
        // The WINC will need to add the offset of the flash were the certificates are stored to this address
        header->u32NextWriteAddr = sizeof(*header); // Next cert will be written after the header
    }

    if (header->u32nEntries >= sizeof(header->astrEntries) / sizeof(header->astrEntries[0]))
    {
        return M2M_ERR_FAIL; // Already at max number of files

    }
    if ((header->u32NextWriteAddr + file_size) > buffer_size)
    {
        return M2M_ERR_FAIL; // Not enough space in buffer for new file

    }
    file_entry = &header->astrEntries[header->u32nEntries];
    header->u32nEntries++;

    if (str_size > sizeof(file_entry->acFileName))
    {
        return M2M_ERR_FAIL; // File name too long
    }
    memcpy((uint8_t*)file_entry->acFileName, (uint8_t*)file_name, str_size);

    file_entry->u32FileSize = file_size;
    file_entry->u32FileAddr = header->u32NextWriteAddr;
    header->u32NextWriteAddr += file_size;

    // Use memmove to accommodate optimizations where the file data is temporarily stored
    // in buffer32
    memmove(((uint8_t*)buffer32) + (file_entry->u32FileAddr), (uint8_t*)file_data, file_size);

    return M2M_SUCCESS;
}

int8_t ecc_set_certificates(WOLFSSL_CTX *ctx)
{
    int8_t status = M2M_SUCCESS;
    int atca_status = ATCACERT_E_SUCCESS;
    uint8_t *signer_cert = NULL;
    size_t signer_cert_size;
#ifdef CLOUD_CONNECT_WITH_CUSTOM_CERTS
    uint8_t signer_public_key[SIGNER_PUBLIC_KEY_MAX_LEN];
#endif
    uint8_t *device_cert = NULL;
    size_t device_cert_size;
    uint8_t cert_sn[CERT_SN_MAX_LEN];
    size_t cert_sn_size;
    uint8_t *file_list = NULL;
    char *device_cert_filename = NULL;
    char *signer_cert_filename = NULL;
    uint32_t sector_buffer[MAX_TLS_CERT_LENGTH];
    char pem_cert[1024];
    size_t pem_cert_size;
    const atcacert_def_t *device_cert_def, *signer_cert_def;

    do
    {
        // Clear cert chain buffer
        memset(sector_buffer, 0xFF, sizeof(sector_buffer));

        // Use the end of the sector buffer to temporarily hold the data to save RAM
        file_list   = ((uint8_t*)sector_buffer) + (sizeof(sector_buffer) - TLS_FILE_NAME_MAX * 2);
        signer_cert = file_list - SIGNER_CERT_MAX_LEN;
        device_cert = signer_cert - DEVICE_CERT_MAX_LEN;

        // Init the file list
        memset(file_list, 0, TLS_FILE_NAME_MAX * 2);
        device_cert_filename = (char*)&file_list[0];
        signer_cert_filename = (char*)&file_list[TLS_FILE_NAME_MAX];

        // Uncompress the signer certificate from the ATECCx08A device
        signer_cert_size = SIGNER_CERT_MAX_LEN;

        if(ATCACERT_E_SUCCESS != (atca_status = atcacert_read_cert(&g_cert_def_1_signer, g_cert_ca_public_key_1_signer,
                                        signer_cert, &signer_cert_size)))
        {
            // Break the do/while loop            
            break;
        }
        pem_cert_size = sizeof(pem_cert);
        atcacert_encode_pem_cert(signer_cert, signer_cert_size, pem_cert, &pem_cert_size);
        APP_DebugPrintf("Signer Cert: \r\n%s\r\n", pem_cert);

        // Get the signer's public key from its certificate
        if(ATCACERT_E_SUCCESS != (atca_status = atcacert_get_subj_public_key(&g_cert_def_1_signer, signer_cert,
                                        signer_cert_size, signer_public_key)))
        {
            // Break the do/while loop            
            break;
        }

        // Uncompress the device certificate from the ATECCx08A device.
        device_cert_size = DEVICE_CERT_MAX_LEN;
        if(ATCACERT_E_SUCCESS != (atca_status = atcacert_read_cert(&g_cert_def_3_device, signer_public_key,
                                        device_cert, &device_cert_size)))
        {
            // Break the do/while loop
            break;
        }
        atcacert_encode_pem_cert(device_cert, device_cert_size, pem_cert, &pem_cert_size);
        APP_DebugPrintf("Device Cert: \r\n%s\r\n", pem_cert);

        if(ATCACERT_E_SUCCESS != (atca_status = atcacert_get_subj_key_id(&g_cert_def_3_device, device_cert,
                                        device_cert_size, subject_key_id)))
        {
            // Break the do/while loop
            break;
        }    

        signer_cert_def = &g_cert_def_1_signer;
        device_cert_def = &g_cert_def_3_device;

        // Get the device certificate SN for the filename
        cert_sn_size = sizeof(cert_sn);
        if(ATCACERT_E_SUCCESS != (atca_status = atcacert_get_cert_sn(device_cert_def, device_cert,
                                           device_cert_size, cert_sn, &cert_sn_size)))
        {
            // Break the do/while loop
            break;
        }

        // Build the device certificate filename
        strcpy(device_cert_filename, "CERT_");
        strcat(device_cert_filename, bin2hex(cert_sn, cert_sn_size));

        // Add the DER device certificate the TLS certs buffer
        status = certs_append_file_buf(sector_buffer, sizeof(sector_buffer),
                                       device_cert_filename, device_cert_size, device_cert);
        if (status != M2M_SUCCESS)
        {
            // Break the do/while loop
            break;
        }

        device_cert = NULL; // Make sure we don't use this now that it has moved

        // Get the signer certificate SN for the filename
        cert_sn_size = sizeof(cert_sn);
        if(ATCACERT_E_SUCCESS != (atca_status = atcacert_get_cert_sn(signer_cert_def, signer_cert,
                                           signer_cert_size, cert_sn, &cert_sn_size)))
        {
            // Break the do/while loop
            break;
        }

        // Build the signer certificate filename
        strcpy(signer_cert_filename, "CERT_");
        strcat(signer_cert_filename, bin2hex(cert_sn, cert_sn_size));

        // Add the DER signer certificate the TLS certs buffer
        status = certs_append_file_buf(sector_buffer, sizeof(sector_buffer),
                                       signer_cert_filename, signer_cert_size, signer_cert);
        if (status != M2M_SUCCESS)
        {
            // Break the do/while loop
            break;
        }

        // Add the cert chain list file to the TLS certs buffer
        status = certs_append_file_buf(sector_buffer, sizeof(sector_buffer),
                                       TLS_SRV_ECDSA_CHAIN_FILE, TLS_FILE_NAME_MAX * 2, file_list);
        if (status != M2M_SUCCESS)
        {
            // Break the do/while loop
            break;
        }

        // Update the TLS cert chain
        SYS_CONSOLE_MESSAGE(TERM_YELLOW"Storing certificates to buffer ... "TERM_RESET);
        device_cert = (uint8_t*)(sector_buffer) + ((tstrTlsSrvSecHdr*)sector_buffer)->astrEntries[0].u32FileAddr;
        status = wolfSSL_CTX_use_certificate_chain_buffer_format(ctx, (const unsigned char*)device_cert,
                 (uint32_t)(signer_cert_size + device_cert_size), WOLFSSL_FILETYPE_ASN1);

        SYS_CONSOLE_MESSAGE(TERM_YELLOW"done\r\n"TERM_RESET);

        file_list = NULL;
        signer_cert_filename = NULL;
        device_cert_filename = NULL;

        if (status != WOLFSSL_SUCCESS)
        {
            SYS_CONSOLE_MESSAGE("Error registering certificate chain: %d\r\n");
            // Break the do/while loop
            break;
        }
    }
    while (false);

    if (atca_status)
    {
        M2M_ERR("eccSetCertificates() failed with ret=%d", atca_status);
        status =  M2M_ERR_FAIL;
    }

    return status;
}

#endif
