/*
 * Copyright (C) 2016 PrasannaKumar Muralidharan <prasannatsmkumar@xxxxxxxxx>
 * Copyright (C) 2020 Corellium LLC
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/printk.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/dma-mapping.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/goldfish.h>
#include <linux/of.h>

#define WRITE_BUFFER_SIZE       16384
#define READ_BUFFER_SIZE        16384
#define COMBINED_BUFFER_SIZE    (2 * (WRITE_BUFFER_SIZE) + 2 * (READ_BUFFER_SIZE))

enum {
        /* audio status register */
        AUDIO_INT_STATUS        = 0x00,
        /* set this to enable IRQ */
        AUDIO_INT_ENABLE        = 0x04,
        /* set these to specify buffer addresses */
        AUDIO_SET_WRITE_BUFFER_1 = 0x08,
        AUDIO_SET_WRITE_BUFFER_2 = 0x0C,
        /* set number of bytes in buffer to write */
        AUDIO_WRITE_BUFFER_1  = 0x10,
        AUDIO_WRITE_BUFFER_2  = 0x14,
        AUDIO_SET_WRITE_BUFFER_1_HIGH = 0x28,
        AUDIO_SET_WRITE_BUFFER_2_HIGH = 0x30,

        /* true if audio input is supported */
        AUDIO_READ_SUPPORTED = 0x18,
        /* buffer to use for audio input */
        AUDIO_SET_READ_BUFFER = 0x1C,
        AUDIO_SET_READ_BUFFER_HIGH = 0x34,

        /* driver writes number of bytes to read */
        AUDIO_START_READ  = 0x20,

        /* number of bytes available in read buffer */
        AUDIO_READ_BUFFER_AVAILABLE  = 0x24,

        /* Corellium-specific way to set frame rate */
        AUDIO_SET_FRAME_RATE  = 0x80,
        AUDIO_SET_READ_FRAME_RATE  = 0x84,
        AUDIO_SET_READ_FRAME_CHANS  = 0x88,

        /* AUDIO_INT_STATUS bits */

        /* this bit set when it is safe to write more bytes to the buffer */
        AUDIO_INT_WRITE_BUFFER_1_EMPTY  = 1U << 0,
        AUDIO_INT_WRITE_BUFFER_2_EMPTY  = 1U << 1,
        AUDIO_INT_READ_BUFFER_FULL      = 1U << 2,

        AUDIO_INT_MASK                  = AUDIO_INT_WRITE_BUFFER_1_EMPTY |
                                          AUDIO_INT_WRITE_BUFFER_2_EMPTY,
};

#define AUDIO_READ(data, addr)          (readl(data->reg_base + addr))
#define AUDIO_WRITE(data, addr, x)      (writel(x, data->reg_base + addr))
#define AUDIO_WRITE64(data, addr, addr2, x)     \
        (gf_write_dma_addr((x), data->reg_base + addr, data->reg_base + addr2))

struct goldfish_audio {
        char __iomem *reg_base;
        int irq;

        /* lock protects access to device registers */
        spinlock_t lock;
        unsigned int_enable;

        struct snd_card *card;
        struct snd_pcm_substream *substream;
        unsigned long frame_rate;
        unsigned long pointer;
        int stream_data;

        struct snd_pcm_substream *read_substream;
        unsigned long read_frame_rate, read_frame_chans;
        unsigned long read_pointer, read_pointer_next;
        int stream_read_data, prime_read_data;
};

static struct snd_pcm_hardware goldfish_pcm_hw = {
        .info = SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER | SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID,
        .formats = SNDRV_PCM_FMTBIT_S16_LE,
        .rates = SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000,
        .rate_min = 44100,
        .rate_max = 48000,
        .channels_min = 2,
        .channels_max = 2,
        .buffer_bytes_max = 2 * 16384,
        .period_bytes_min = 1,
        .period_bytes_max = 16384,
        .periods_min = 2,
        .periods_max = 4,
};

static int goldfish_pcm_open(struct snd_pcm_substream *substream)
{
        struct snd_pcm_runtime *runtime = substream->runtime;
        struct goldfish_audio *audio_data = substream->private_data;
        unsigned long irq_flags;

        runtime->hw = goldfish_pcm_hw;

        spin_lock_irqsave(&audio_data->lock, irq_flags);
        audio_data->stream_data = false;
        audio_data->substream = substream;
        spin_unlock_irqrestore(&audio_data->lock, irq_flags);

        return 0;
}

static int goldfish_pcm_hw_params(struct snd_pcm_substream *substream,
                                  struct snd_pcm_hw_params *hw_params)
{
        int ret;

        ret = snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(hw_params));
        if(ret < 0)
            pr_err("goldfish_pcm_hw_params(), snd_pcm_lib_malloc_pages() failed\n");

        return ret;
}

static int goldfish_pcm_hw_free(struct snd_pcm_substream *substream)
{
        int ret;

        ret = snd_pcm_lib_free_pages(substream);
        if (ret < 0)
            pr_err("goldfish_pcm_hw_free(), snd_pcm_lib_free_pages() failed\n");

        return ret;
}

static int goldfish_pcm_prepare(struct snd_pcm_substream *substream)
{
        /* Nothing to do here */
        return 0;
}

static int goldfish_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
        struct goldfish_audio *audio_data = substream->private_data;
        unsigned long irq_flags;

        switch (cmd) {
        case SNDRV_PCM_TRIGGER_START:
                spin_lock_irqsave(&audio_data->lock, irq_flags);
                audio_data->stream_data = true;
                audio_data->int_enable |= AUDIO_INT_MASK;
                AUDIO_WRITE(audio_data, AUDIO_INT_ENABLE, audio_data->int_enable);
                spin_unlock_irqrestore(&audio_data->lock, irq_flags);
                break;
        case SNDRV_PCM_TRIGGER_STOP:
                spin_lock_irqsave(&audio_data->lock, irq_flags);
                audio_data->int_enable &= ~AUDIO_INT_MASK;
                AUDIO_WRITE(audio_data, AUDIO_INT_ENABLE, audio_data->int_enable);
                audio_data->stream_data = false;
                spin_unlock_irqrestore(&audio_data->lock, irq_flags);
                break;
        default:
                pr_info("goldfish_pcm_trigger(), invalid\n");
                return -EINVAL;
        }

        return 0;
}

static snd_pcm_uframes_t goldfish_pcm_pointer(struct snd_pcm_substream *s)
{
        struct goldfish_audio *audio_data = s->private_data;
        snd_pcm_uframes_t pos = 0;

        pos = audio_data->pointer;
        return pos;
}

static int goldfish_pcm_close(struct snd_pcm_substream *substream)
{
        struct goldfish_audio *audio_data = substream->private_data;
        unsigned long irq_flags;

        spin_lock_irqsave(&audio_data->lock, irq_flags);
        audio_data->int_enable &= ~AUDIO_INT_MASK;
        AUDIO_WRITE(audio_data, AUDIO_INT_ENABLE, audio_data->int_enable);
        audio_data->stream_data = false;
        audio_data->substream = NULL;
        spin_unlock_irqrestore(&audio_data->lock, irq_flags);

        return 0;
}

static struct snd_pcm_ops goldfish_pcm_ops = {
        .open       = goldfish_pcm_open,
        .close      = goldfish_pcm_close,
        .ioctl      = snd_pcm_lib_ioctl,
        .hw_params  = goldfish_pcm_hw_params,
        .hw_free    = goldfish_pcm_hw_free,
        .prepare    = goldfish_pcm_prepare,
        .trigger    = goldfish_pcm_trigger,
        .pointer    = goldfish_pcm_pointer,
};

static struct snd_pcm_hardware goldfish_read_pcm_hw = {
        .info = SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER | SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID,
        .formats = SNDRV_PCM_FMTBIT_S16_LE,
        .rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000,
        .rate_min = 8000,
        .rate_max = 48000,
        .channels_min = 1,
        .channels_max = 2,
        .buffer_bytes_max = 2 * 16384,
        .period_bytes_min = 1,
        .period_bytes_max = 16384,
        .periods_min = 2,
        .periods_max = 4,
};

static int goldfish_read_pcm_open(struct snd_pcm_substream *substream)
{
        struct snd_pcm_runtime *runtime = substream->runtime;
        struct goldfish_audio *audio_data = substream->private_data;
        unsigned long irq_flags;

        runtime->hw = goldfish_read_pcm_hw;

        spin_lock_irqsave(&audio_data->lock, irq_flags);
        audio_data->stream_read_data = false;
        audio_data->read_substream = substream;
        spin_unlock_irqrestore(&audio_data->lock, irq_flags);

        return 0;
}

static int goldfish_read_pcm_hw_params(struct snd_pcm_substream *substream,
                                  struct snd_pcm_hw_params *hw_params)
{
        int ret;

        ret = snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(hw_params));
        if(ret < 0)
            pr_err("goldfish_read_pcm_hw_params(), snd_pcm_lib_malloc_pages() failed\n");

        return ret;
}

static int goldfish_read_pcm_hw_free(struct snd_pcm_substream *substream)
{
        int ret;

        ret = snd_pcm_lib_free_pages(substream);
        if (ret < 0)
            pr_err("goldfish_read_pcm_hw_free(), snd_pcm_lib_free_pages() failed\n");

        return ret;
}

static int goldfish_read_pcm_prepare(struct snd_pcm_substream *substream)
{
        /* Nothing to do here */
        return 0;
}

static int goldfish_read_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
        struct goldfish_audio *audio_data = substream->private_data;
        unsigned long irq_flags;
        struct snd_pcm_runtime *runtime = substream->runtime;

        switch (cmd) {
        case SNDRV_PCM_TRIGGER_START:
                spin_lock_irqsave(&audio_data->lock, irq_flags);
                if(runtime) {
                    if(runtime->rate != audio_data->read_frame_rate) {
                        AUDIO_WRITE(audio_data, AUDIO_SET_READ_FRAME_RATE, runtime->rate);
                        audio_data->read_frame_rate = runtime->rate;
                    }
                    if(runtime->channels != audio_data->read_frame_chans) {
                        AUDIO_WRITE(audio_data, AUDIO_SET_READ_FRAME_CHANS, runtime->channels);
                        audio_data->read_frame_chans = runtime->channels;
                    }
                }
                audio_data->stream_read_data = true;
                audio_data->prime_read_data = true;
                audio_data->int_enable |= AUDIO_INT_READ_BUFFER_FULL;
                AUDIO_WRITE(audio_data, AUDIO_INT_ENABLE, audio_data->int_enable);
                AUDIO_WRITE(audio_data, AUDIO_START_READ, 0);
                spin_unlock_irqrestore(&audio_data->lock, irq_flags);
                break;
        case SNDRV_PCM_TRIGGER_STOP:
                spin_lock_irqsave(&audio_data->lock, irq_flags);
                audio_data->stream_read_data = true;
                audio_data->int_enable &= ~AUDIO_INT_READ_BUFFER_FULL;
                AUDIO_WRITE(audio_data, AUDIO_INT_ENABLE, audio_data->int_enable);
                AUDIO_WRITE(audio_data, AUDIO_START_READ, 0);
                spin_unlock_irqrestore(&audio_data->lock, irq_flags);
                break;
        default:
                pr_info("goldfish_pcm_trigger(), invalid\n");
                return -EINVAL;
        }

        return 0;
}

static snd_pcm_uframes_t goldfish_read_pcm_pointer(struct snd_pcm_substream *s)
{
        struct goldfish_audio *audio_data = s->private_data;
        snd_pcm_uframes_t pos = 0;

        pos = audio_data->read_pointer;
        return pos;
}

static int goldfish_read_pcm_close(struct snd_pcm_substream *substream)
{
        struct goldfish_audio *audio_data = substream->private_data;
        unsigned long irq_flags;

        spin_lock_irqsave(&audio_data->lock, irq_flags);
        audio_data->int_enable &= ~AUDIO_INT_READ_BUFFER_FULL;
        AUDIO_WRITE(audio_data, AUDIO_INT_ENABLE, audio_data->int_enable);
        AUDIO_WRITE(audio_data, AUDIO_START_READ, 0);
        audio_data->stream_read_data = false;
        audio_data->read_substream = NULL;
        spin_unlock_irqrestore(&audio_data->lock, irq_flags);

        return 0;
}

static struct snd_pcm_ops goldfish_read_pcm_ops = {
        .open       = goldfish_read_pcm_open,
        .close      = goldfish_read_pcm_close,
        .ioctl      = snd_pcm_lib_ioctl,
        .hw_params  = goldfish_read_pcm_hw_params,
        .hw_free    = goldfish_read_pcm_hw_free,
        .prepare    = goldfish_read_pcm_prepare,
        .trigger    = goldfish_read_pcm_trigger,
        .pointer    = goldfish_read_pcm_pointer,
};

static irqreturn_t goldfish_audio_interrupt(int irq, void *data)
{
        unsigned long irq_flags, buf_addr, buf_size, buf_max;
        struct goldfish_audio *audio_data = data;
        struct snd_pcm_substream *substream = NULL, *read_substream = NULL;
        u32 status;

        spin_lock_irqsave(&audio_data->lock, irq_flags);

        status = AUDIO_READ(audio_data, AUDIO_INT_STATUS);
        AUDIO_WRITE(audio_data, AUDIO_INT_ENABLE, 0);
        status &= AUDIO_INT_MASK | AUDIO_INT_READ_BUFFER_FULL;

        if (!status) {
            spin_unlock_irqrestore(&audio_data->lock, irq_flags);
            return IRQ_NONE;
        }

        if (status & AUDIO_INT_MASK) {
            audio_data->int_enable &= ~AUDIO_INT_MASK;

            if(audio_data->stream_data)
                substream = audio_data->substream;

            if(substream) {
                struct snd_pcm_runtime *runtime = substream->runtime;
                buf_addr = runtime->dma_addr + audio_data->pointer * 4;
                buf_size = runtime->period_size * 4;
                buf_max = runtime->buffer_size * 4;
                if(buf_size > buf_max - audio_data->pointer * 4)
                    buf_size = buf_max - audio_data->pointer * 4;
                if(runtime->rate != audio_data->frame_rate) {
                    AUDIO_WRITE(audio_data, AUDIO_SET_FRAME_RATE, runtime->rate);
                    audio_data->frame_rate = runtime->rate;
                }
            } else {
                buf_addr = buf_size = 0;
                buf_max = 4;
            }

            /*
             *  clear the buffer empty flag, and signal the emulator
             *  to start writing the buffer
             */
            if(buf_size) {
                if (status & AUDIO_INT_WRITE_BUFFER_1_EMPTY) {
                    AUDIO_WRITE64(audio_data, AUDIO_SET_WRITE_BUFFER_1,
                            AUDIO_SET_WRITE_BUFFER_1_HIGH, buf_addr);
                    AUDIO_WRITE(audio_data, AUDIO_WRITE_BUFFER_1, buf_size);
                    audio_data->int_enable |= AUDIO_INT_MASK;
                } else if (status & AUDIO_INT_WRITE_BUFFER_2_EMPTY) {
                    AUDIO_WRITE64(audio_data, AUDIO_SET_WRITE_BUFFER_2,
                            AUDIO_SET_WRITE_BUFFER_2_HIGH, buf_addr);
                    AUDIO_WRITE(audio_data, AUDIO_WRITE_BUFFER_2, buf_size);
                    audio_data->int_enable |= AUDIO_INT_MASK;
                }
                audio_data->pointer += buf_size >> 2;
                audio_data->pointer %= buf_max >> 2;
            }
        }

        if (status & AUDIO_INT_READ_BUFFER_FULL) {
            audio_data->int_enable &= ~AUDIO_INT_READ_BUFFER_FULL;

            if(audio_data->stream_read_data)
                read_substream = audio_data->read_substream;

            audio_data->read_pointer = audio_data->read_pointer_next;

            if(read_substream) {
                struct snd_pcm_runtime *runtime = read_substream->runtime;
                buf_addr = runtime->dma_addr + audio_data->read_pointer * 2 * runtime->channels;
                buf_size = runtime->period_size * 2 * runtime->channels;
                buf_max = runtime->buffer_size * 2 * runtime->channels;
                if(buf_size > buf_max - audio_data->read_pointer * 2 * runtime->channels)
                    buf_size = buf_max - audio_data->read_pointer * 2 * runtime->channels;
                if(runtime->rate != audio_data->read_frame_rate) {
                    AUDIO_WRITE(audio_data, AUDIO_SET_READ_FRAME_RATE, runtime->rate);
                    audio_data->read_frame_rate = runtime->rate;
                }
                if(runtime->channels != audio_data->read_frame_chans) {
                    AUDIO_WRITE(audio_data, AUDIO_SET_READ_FRAME_CHANS, runtime->channels);
                    audio_data->read_frame_chans = runtime->channels;
                }
            } else {
                buf_addr = buf_size = 0;
                buf_max = 4;
            }

            if(buf_size) {
                AUDIO_WRITE64(audio_data, AUDIO_SET_READ_BUFFER,
                        AUDIO_SET_READ_BUFFER_HIGH, buf_addr);
                AUDIO_WRITE(audio_data, AUDIO_START_READ, buf_size);
                audio_data->int_enable |= AUDIO_INT_READ_BUFFER_FULL;
                if(audio_data->read_frame_chans) {
                    audio_data->read_pointer_next += buf_size / (2 * audio_data->read_frame_chans);
                    audio_data->read_pointer_next %= buf_max / (2 * audio_data->read_frame_chans);
                }
            }

            if(audio_data->prime_read_data) {
                read_substream = NULL;
                audio_data->prime_read_data = 0;
            }
        }

        AUDIO_WRITE(audio_data, AUDIO_INT_ENABLE, audio_data->int_enable);
        spin_unlock_irqrestore(&audio_data->lock, irq_flags);

        if(substream)
            snd_pcm_period_elapsed(substream);
        if(read_substream)
            snd_pcm_period_elapsed(read_substream);

        return IRQ_HANDLED;
}

static int goldfish_audio_probe(struct platform_device *pdev)
{
        int ret, record;
        struct resource *r;
        struct goldfish_audio *data;
        struct snd_card *card;
        struct snd_pcm *pcm;

        ret = snd_card_new(&pdev->dev, -1, "eac", THIS_MODULE,
                           sizeof(*data), &card);
        if (ret < 0)
                return ret;

        data = card->private_data;
        data->card = card;
        spin_lock_init(&data->lock);
        platform_set_drvdata(pdev, data);

        r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
        if (!r) {
                dev_err(&pdev->dev, "platform_get_resource failed\n");
                ret = -ENODEV;
                goto fail;
        }
        data->reg_base = devm_ioremap(&pdev->dev, r->start, PAGE_SIZE);
        if (!data->reg_base) {
                ret = -ENOMEM;
                goto fail;
        }

        data->irq = platform_get_irq(pdev, 0);
        if (data->irq < 0) {
                dev_err(&pdev->dev, "platform_get_irq failed\n");
                ret = -ENODEV;
                goto fail;
        }

        ret = devm_request_irq(&pdev->dev, data->irq, goldfish_audio_interrupt,
                               IRQF_SHARED, pdev->name, data);
        if (ret) {
                dev_err(&pdev->dev, "request_irq failed\n");
                goto fail;
        }

        snprintf(card->driver, sizeof(card->driver), "%s", "goldfish_audio");
        snprintf(card->shortname, sizeof(card->shortname), "EAC Goldfish:%d",
                 card->number);
        snprintf(card->longname, sizeof(card->longname), "%s", card->shortname);

        record = !!AUDIO_READ(data, AUDIO_READ_SUPPORTED);
        ret = snd_pcm_new(card, "eac", 0, 1, record, &pcm);
        if (ret < 0)
                goto fail;

        pcm->private_data = data;
        snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &goldfish_pcm_ops);
        if(record)
            snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &goldfish_read_pcm_ops);
        ret = snd_card_register(card);
        if (ret < 0)
                goto fail;

        snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_DEV,
            &pdev->dev, COMBINED_BUFFER_SIZE, COMBINED_BUFFER_SIZE);

        data->stream_data = false;
        data->stream_read_data = false;

        return 0;

fail:
        snd_card_free(card);
        return ret;
}

static int goldfish_audio_remove(struct platform_device *pdev)
{
        struct goldfish_audio *audio_data = platform_get_drvdata(pdev);
        unsigned long irq_flags;

        spin_lock_irqsave(&audio_data->lock, irq_flags);
        snd_card_free(audio_data->card);
        spin_unlock_irqrestore(&audio_data->lock, irq_flags);
        return 0;
}

static const struct of_device_id goldfish_audio_of_match[] = {
        { .compatible = "google,goldfish-audio", },
        {},
};
MODULE_DEVICE_TABLE(of, goldfish_audio_of_match);

static struct platform_driver goldfish_audio_driver = {
        .probe          = goldfish_audio_probe,
        .remove         = goldfish_audio_remove,
        .driver = {
                .name = "goldfish_audio",
                .of_match_table = goldfish_audio_of_match,
        }
};

module_platform_driver(goldfish_audio_driver);

MODULE_AUTHOR("PrasannaKumar Muralidharan <prasannatsmkumar@xxxxxxxxx>");
MODULE_DESCRIPTION("Android QEMU Audio Driver");
MODULE_LICENSE("GPL");
