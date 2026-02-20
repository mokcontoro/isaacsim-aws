declare module 'roslib' {
  import { EventEmitter } from 'eventemitter3';

  export class Ros extends EventEmitter {
    constructor(options?: { url?: string });
    connect(url: string): void;
    close(): void;
    on(event: string, callback: (...args: any[]) => void): this;
    off(event: string, callback: (...args: any[]) => void): this;
    isConnected: boolean;
  }

  export class Topic {
    constructor(options: {
      ros: Ros;
      name: string;
      messageType: string;
      compression?: string;
      throttle_rate?: number;
      queue_size?: number;
      latch?: boolean;
      queue_length?: number;
    });
    publish(message: any): void;
    subscribe(callback: (message: any) => void): void;
    unsubscribe(callback?: (message: any) => void): void;
    advertise(): void;
    unadvertise(): void;
    name: string;
    messageType: string;
  }

  export class Service {
    constructor(options: {
      ros: Ros;
      name: string;
      serviceType: string;
    });
    callService(request: any, callback: (response: any) => void, failedCallback?: (error: any) => void): void;
    advertise(callback: (request: any, response: any) => boolean): void;
    unadvertise(): void;
  }

  export class Param {
    constructor(options: {
      ros: Ros;
      name: string;
    });
    get(callback: (value: any) => void, failedCallback?: (error: any) => void): void;
    set(value: any, callback?: () => void): void;
    delete(callback?: () => void): void;
  }
}
